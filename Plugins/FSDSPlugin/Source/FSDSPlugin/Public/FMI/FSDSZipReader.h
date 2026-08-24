// Minimal ZIP reader for .fmu packages.
#pragma once

#include "CoreMinimal.h"

/**
 * A small, self-contained ZIP reader — just enough to unpack an .fmu.
 *
 * WHY NOT FZipArchiveReader
 * -------------------------
 * The engine ships FZipArchiveReader (Developer/FileUtilities), but its
 * libzip backend is linked only under `if (Target.bBuildEditor)`
 * (FileUtilities.Build.cs). Depending on it would mean FMU loading works in
 * the editor and silently fails in a packaged build — and IFSSIM ships a
 * packaged Mac binary. A plant that loads only under the editor is not a
 * plant, so this reads the container itself using zlib, which is available
 * to every target.
 *
 * SCOPE, deliberately narrow: reads the central directory and extracts
 * entries stored (method 0) or deflated (method 8). Those are the only two
 * methods the FMI standard's packaging permits in practice. Anything else —
 * including encryption and Zip64 — is reported as an explicit error rather
 * than silently mis-read, because a half-extracted FMU would present as a
 * bizarre runtime failure much later.
 */
class FSDSPLUGIN_API FFSDSZipReader
{
public:
	struct FEntry
	{
		FString Name;
		uint32  CompressedSize = 0;
		uint32  UncompressedSize = 0;
		uint16  Method = 0;          // 0 = store, 8 = deflate
		uint32  LocalHeaderOffset = 0;
		bool    bIsDirectory = false;
	};

	/** Open and read the central directory. Check IsValid() afterwards. */
	explicit FFSDSZipReader(const FString& ZipPath);

	bool IsValid() const { return bValid; }
	const FString& GetError() const { return Error; }
	const TArray<FEntry>& GetEntries() const { return Entries; }

	/** Extract one entry by exact name. */
	bool ExtractFile(const FString& Name, TArray<uint8>& OutData) const;

	/**
	 * Extract everything to DestDir, recreating the archive's directory
	 * structure. Returns the number of files written, or -1 on failure.
	 *
	 * Rejects entries whose path escapes DestDir (zip-slip). An .fmu is
	 * a file a student downloads from a teammate; treating it as trusted
	 * input would be a mistake.
	 */
	int32 ExtractAll(const FString& DestDir) const;

private:
	bool ReadCentralDirectory();
	bool Inflate(const TArray<uint8>& In, int32 ExpectedSize, TArray<uint8>& Out) const;

	FString Path;
	TArray<uint8> Buffer;      // whole archive; .fmu files are small enough
	TArray<FEntry> Entries;
	FString Error;
	bool bValid = false;
};
