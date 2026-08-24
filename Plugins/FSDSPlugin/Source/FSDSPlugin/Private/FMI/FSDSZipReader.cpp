#include "FMI/FSDSZipReader.h"

#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

THIRD_PARTY_INCLUDES_START
#include "zlib.h"
THIRD_PARTY_INCLUDES_END

namespace
{
	constexpr uint32 SigEOCD  = 0x06054b50;   // end of central directory
	constexpr uint32 SigCD    = 0x02014b50;   // central directory file header
	constexpr uint32 SigLocal = 0x04034b50;   // local file header
	constexpr uint32 SigEOCD64Locator = 0x07064b50;

	// Little-endian reads. ZIP is defined little-endian regardless of host,
	// so do not use memcpy of a struct — that would break on a big-endian
	// host and, more importantly, on any struct padding.
	uint16 R16(const uint8* P) { return uint16(P[0]) | (uint16(P[1]) << 8); }
	uint32 R32(const uint8* P)
	{
		return uint32(P[0]) | (uint32(P[1]) << 8) | (uint32(P[2]) << 16) | (uint32(P[3]) << 24);
	}
}

FFSDSZipReader::FFSDSZipReader(const FString& ZipPath)
	: Path(ZipPath)
{
	if (!FFileHelper::LoadFileToArray(Buffer, *Path))
	{
		Error = FString::Printf(TEXT("cannot read '%s'"), *Path);
		return;
	}
	if (Buffer.Num() < 22)
	{
		Error = TEXT("file is too small to be a zip archive");
		return;
	}
	bValid = ReadCentralDirectory();
}

bool FFSDSZipReader::ReadCentralDirectory()
{
	// The EOCD sits at the very end, after a variable-length comment of up to
	// 65535 bytes. Scan backwards for its signature.
	const int32 MaxComment = 65535;
	const int32 ScanFrom = FMath::Max(0, Buffer.Num() - (MaxComment + 22));
	int32 EOCD = INDEX_NONE;
	for (int32 i = Buffer.Num() - 22; i >= ScanFrom; --i)
	{
		if (R32(&Buffer[i]) == SigEOCD) { EOCD = i; break; }
	}
	if (EOCD == INDEX_NONE)
	{
		Error = TEXT("no end-of-central-directory record — not a zip archive");
		return false;
	}

	const uint16 NumEntries = R16(&Buffer[EOCD + 10]);
	const uint32 CDOffset   = R32(&Buffer[EOCD + 16]);

	// Zip64 uses sentinel values in the 32-bit fields. Refuse loudly rather
	// than reading a truncated offset and producing nonsense entries.
	if (CDOffset == 0xFFFFFFFFu || NumEntries == 0xFFFFu)
	{
		Error = TEXT("Zip64 archive — not supported; re-export the FMU without Zip64");
		return false;
	}
	if (EOCD >= 20 && R32(&Buffer[EOCD - 20]) == SigEOCD64Locator)
	{
		Error = TEXT("Zip64 locator present — not supported");
		return false;
	}
	if (CDOffset >= uint32(Buffer.Num()))
	{
		Error = TEXT("central directory offset is past end of file (truncated?)");
		return false;
	}

	Entries.Reserve(NumEntries);
	uint32 Cursor = CDOffset;

	for (uint16 n = 0; n < NumEntries; ++n)
	{
		if (Cursor + 46 > uint32(Buffer.Num()) || R32(&Buffer[Cursor]) != SigCD)
		{
			Error = FString::Printf(TEXT("malformed central directory at entry %u"), n);
			return false;
		}

		FEntry E;
		const uint16 Flags   = R16(&Buffer[Cursor + 8]);
		E.Method             = R16(&Buffer[Cursor + 10]);
		E.CompressedSize     = R32(&Buffer[Cursor + 20]);
		E.UncompressedSize   = R32(&Buffer[Cursor + 24]);
		const uint16 NameLen = R16(&Buffer[Cursor + 28]);
		const uint16 ExtraLen= R16(&Buffer[Cursor + 30]);
		const uint16 CommLen = R16(&Buffer[Cursor + 32]);
		E.LocalHeaderOffset  = R32(&Buffer[Cursor + 42]);

		if (Flags & 0x1)
		{
			Error = TEXT("archive contains encrypted entries");
			return false;
		}

		const uint8* NameBytes = &Buffer[Cursor + 46];
		// ZIP names are UTF-8 when bit 11 is set, CP437 otherwise. Every FMU
		// in practice is ASCII; treat as UTF-8, which is correct for both.
		//
		// USE THE LENGTH-AWARE FSTRING CONSTRUCTOR. This previously read
		//     FString(FUTF8ToTCHAR(Bytes, NameLen).Get(), NameLen)
		// which looks like (pointer, length) but is not: FString's two-argument
		// form is (Src, ExtraSlack). NameLen was silently taken as extra
		// capacity, and the name itself was read as a NUL-terminated string out
		// of a conversion buffer that is NOT NUL-terminated when built with an
		// explicit length. Every entry name picked up trailing garbage, so
		// nothing ever matched and the archive looked empty of known files.
		// It survived my hand-built fixtures only because those were checked
		// with the Python inspector and never through this code path.
		FUTF8ToTCHAR NameConv(reinterpret_cast<const ANSICHAR*>(NameBytes), NameLen);
		E.Name = FString(NameConv.Length(), NameConv.Get());
		E.bIsDirectory = E.Name.EndsWith(TEXT("/"));

		Entries.Add(MoveTemp(E));
		Cursor += 46u + NameLen + ExtraLen + CommLen;
	}

	return true;
}

bool FFSDSZipReader::Inflate(const TArray<uint8>& In, int32 ExpectedSize, TArray<uint8>& Out) const
{
	Out.SetNumUninitialized(ExpectedSize);
	if (ExpectedSize == 0) return true;

	z_stream S;
	FMemory::Memzero(S);
	S.next_in   = const_cast<Bytef*>(In.GetData());
	S.avail_in  = In.Num();
	S.next_out  = Out.GetData();
	S.avail_out = ExpectedSize;

	// Negative window bits selects RAW deflate — zip entries carry no zlib
	// header. Getting this wrong yields Z_DATA_ERROR on every entry.
	if (inflateInit2(&S, -MAX_WBITS) != Z_OK) return false;
	const int R = inflate(&S, Z_FINISH);
	inflateEnd(&S);

	return (R == Z_STREAM_END) && (S.total_out == uLong(ExpectedSize));
}

bool FFSDSZipReader::ExtractFile(const FString& Name, TArray<uint8>& OutData) const
{
	if (!bValid) return false;

	const FEntry* E = Entries.FindByPredicate(
		[&Name](const FEntry& X) { return X.Name == Name; });
	if (!E || E->bIsDirectory) return false;

	// The central directory's name/extra lengths are NOT authoritative for the
	// local header — they are allowed to differ. Re-read them from the local
	// header or the data offset will be wrong for some writers.
	const uint32 LH = E->LocalHeaderOffset;
	if (LH + 30 > uint32(Buffer.Num()) || R32(&Buffer[LH]) != SigLocal) return false;

	const uint16 LocalNameLen  = R16(&Buffer[LH + 26]);
	const uint16 LocalExtraLen = R16(&Buffer[LH + 28]);
	const uint32 DataOffset    = LH + 30u + LocalNameLen + LocalExtraLen;

	if (DataOffset + E->CompressedSize > uint32(Buffer.Num())) return false;

	if (E->Method == 0)
	{
		if (E->CompressedSize != E->UncompressedSize) return false;
		OutData.SetNumUninitialized(E->UncompressedSize);
		FMemory::Memcpy(OutData.GetData(), &Buffer[DataOffset], E->UncompressedSize);
		return true;
	}
	if (E->Method == 8)
	{
		TArray<uint8> Raw;
		Raw.SetNumUninitialized(E->CompressedSize);
		FMemory::Memcpy(Raw.GetData(), &Buffer[DataOffset], E->CompressedSize);
		return Inflate(Raw, E->UncompressedSize, OutData);
	}
	return false;   // any other method: unsupported, and we say so rather than guess
}

int32 FFSDSZipReader::ExtractAll(const FString& DestDir) const
{
	if (!bValid) return -1;

	IFileManager& FM = IFileManager::Get();
	const FString Root = FPaths::ConvertRelativePathToFull(DestDir);
	FM.MakeDirectory(*Root, /*Tree=*/true);

	int32 Written = 0;
	for (const FEntry& E : Entries)
	{
		if (E.bIsDirectory) continue;

		// Zip-slip guard. An .fmu is a file one student hands another; an
		// entry named "../../.." must not be able to write outside DestDir.
		FString Target = FPaths::ConvertRelativePathToFull(FPaths::Combine(Root, E.Name));
		if (!Target.StartsWith(Root))
		{
			UE_LOG(LogTemp, Error,
				TEXT("FSDS FMU: refusing entry '%s' — it escapes the extraction directory"),
				*E.Name);
			return -1;
		}

		TArray<uint8> Data;
		if (!ExtractFile(E.Name, Data))
		{
			UE_LOG(LogTemp, Error,
				TEXT("FSDS FMU: failed to extract '%s' (method %u, %u -> %u bytes)"),
				*E.Name, E.Method, E.CompressedSize, E.UncompressedSize);
			return -1;
		}

		FM.MakeDirectory(*FPaths::GetPath(Target), /*Tree=*/true);
		if (!FFileHelper::SaveArrayToFile(Data, *Target))
		{
			UE_LOG(LogTemp, Error, TEXT("FSDS FMU: cannot write '%s'"), *Target);
			return -1;
		}
		++Written;
	}
	return Written;
}
