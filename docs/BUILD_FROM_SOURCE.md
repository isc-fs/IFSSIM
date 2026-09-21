# Building IFSSIM from source

**For people who have never touched Unreal Engine, ROS, or a compiler.**
You do not need to understand any of it. You need to install one big
program, then run one command.

There are no prebuilt downloads for v0.2.0, so this is currently the
only way to get the sim running.

| | |
|---|---|
| **Time** | ~30 min the first time (~25 of it installing Unreal), ~1 min after |
| **Disk** | ~80 GB free — 40 for the engine, 40 for the build |
| **You type** | two commands |

---

## The short version

If you already have Unreal Engine 5.7, Git and a compiler:

```bash
./tools/build_sim.sh
```

That is the whole guide. Everything below is for when that is not
already true, or when it goes wrong.

---

## Step 1 — Install Unreal Engine 5.7

**This is the only step you have to do by hand, and it is the slow one.**

The build script installs everything else for you, but it cannot install
Unreal: the engine is ~40 GB and needs an Epic Games account plus a
licence agreement that only you can accept.

### Windows and macOS

1. Download the **Epic Games Launcher**:
   <https://www.epicgames.com/store/download>
2. Install it, then sign in (create a free account if you need one).
3. Click the **Unreal Engine** tab on the left, then **Library**.
4. Click the **`+`** next to ENGINE VERSIONS.
5. Change the version dropdown to **5.7**, click **Install**.
6. Accept the defaults. Go and do something else for ~20 minutes.

> **Do not install 5.6 or 5.8.** The project is pinned to 5.7
> (`IFSSIM.uproject`). A different engine will not open it.

### Linux

Epic ships no Linux binary — the engine has to be compiled:

1. Link your GitHub account to Epic: <https://www.unrealengine.com/ue-on-github>
2. ```bash
   git clone -b 5.7 https://github.com/EpicGames/UnrealEngine
   cd UnrealEngine && ./Setup.sh && ./GenerateProjectFiles.sh && make
   ```
   (~1 hour on 16 cores.)
3. Remember where you put it — you will pass it as `UE_ROOT` in step 3.

---

## Step 2 — Get the code

You need **Git**. If you do not have it:

- **Windows** — <https://git-scm.com/download/win>, accept every default.
  (Or skip this: the PowerShell script in step 3 installs Git for you.)
- **macOS** — `xcode-select --install`, or `brew install git`.
- **Linux** — `sudo apt install git` (or your distro's equivalent).

Then, in a terminal:

```bash
git clone https://github.com/isc-fs/IFSSIM.git
cd IFSSIM
git fetch origin dev
git checkout dev
```

> ### Those last two lines are not optional
> The repository's default branch is `main`, which is an intentionally
> empty placeholder — it holds a single README and nothing else. All the
> code, including the build script below, lives on `dev`. Clone without
> checking out `dev` and step 3 fails with "no such file or directory".
>
> Check you got it: `ls` should show `Content`, `Plugins`, `docs` and
> `tools`. If all you see is `README.md`, you are still on `main` — run
> `git checkout dev`.

> Don't worry about `git lfs` or `git submodule` — the build script
> checks both and fixes them for you.

**Windows tip:** clone somewhere short and simple like
`C:\dev\IFSSIM`. Deep paths under `Documents` or OneDrive have caused
build failures — Windows still has a 260-character path limit and
Unreal's intermediate filenames are long.

---

## Step 3 — Build

### macOS and Linux

```bash
./tools/build_sim.sh
```

### Windows

Right-click **`tools\build_sim.ps1`** → **Run with PowerShell**.

Or, from a terminal:

```powershell
powershell -ExecutionPolicy Bypass -File tools\build_sim.ps1
```

If you already have Git Bash open, you can skip PowerShell entirely and
run the same script the other platforms use:

```bash
./tools/build_sim.sh
```

### What it does

It walks six steps and prints `OK` / `WARN` / `MISSING` for each:

1. **Machine** — detects your OS and checks you have the disk space
2. **Package manager** — finds Homebrew / winget / apt (offers to install Homebrew)
3. **Build tools** — checks git, git-lfs, and your compiler; installs what it can
4. **Unreal Engine** — searches every standard install location
5. **Repository** — downloads the LFS assets and initialises the submodules
6. **Build** — runs the right `package_*.sh` for your platform

Anything it can fix, it fixes. Anything it cannot, it explains — with
the exact link or command you need.

### Useful flags

| Flag | What it does |
|---|---|
| `--check` | Check everything, change nothing, build nothing. **Run this first if you are nervous.** |
| `--yes` | Never ask; install whatever is missing |
| `--fast` | Skip building the ~1 GB distributable zip (saves ~30 s) |
| `--help` | Show usage |

Installed Unreal somewhere unusual? Point at it:

```bash
UE_ROOT="/path/to/UE_5.7" ./tools/build_sim.sh
```

---

## Step 4 — Run it

When the build finishes it tells you exactly what to run. It will be:

| | |
|---|---|
| **macOS** | `open Saved/StagedBuilds/Mac/IFSSIM-Mac-Shipping.app` |
| **Windows** | double-click `Saved\StagedBuilds\Windows\IFSSIM.exe` |
| **Linux** | `./Saved/StagedBuilds/Linux/IFSSIM.sh` |

**On first launch Windows shows a Firewall prompt** for port 41451 —
click **Allow**. The sim talks to the autonomy stack over that port; if
you block it, nothing will connect.

**The sim opens on an empty map with no track. That is correct.** Tracks
are loaded from Mission Control, not from the sim. To bring that up, see
[SETUP.md](SETUP.md) step 4 — it needs Docker.

---

## When it goes wrong

| What you see | What it means |
|---|---|
| `MISSING Unreal Engine 5.7 not found` | Step 1 is not done, or it went somewhere odd. The script prints every path it looked in — if yours is not listed, pass `UE_ROOT=...`. |
| `Only the Command Line Tools are installed` | macOS. Unreal needs the full Xcode from the App Store, not just `xcode-select --install`. After installing: `sudo xcode-select -s /Applications/Xcode.app/Contents/Developer` |
| `Visual Studio 2022 ... was not found` | Windows. Say yes when the script offers to install it, or install VS 2022 Community by hand with the **"Game development with C++"** workload ticked. |
| `Access to the path 'IFSSIM.exe' is denied` | The sim is still running from last time. Close it, or `Stop-Process -Name IFSSIM*` in PowerShell. |
| Build dies partway with disk errors | You ran out of space mid-cook. The script warns about this at the top — heed it. |
| Assets look wrong / cook fails on a `.uasset` | LFS files did not download. Run `git lfs pull`, then build again. |
| `Disk space: only N GB free` | A cold build wants ~40 GB for `Intermediate/`, the DerivedDataCache, and the staged build. |

Still stuck? Open an issue with the **full output** of:

```bash
./tools/build_sim.sh --check
```

That one command captures your OS, your tool versions, where your engine
is, and the repo's health — which is most of what anyone needs to help
you.

---

## What about the autonomy stack?

This guide gets you a **driving simulator you can steer with a
keyboard**. That is a complete, useful thing on its own.

Wiring up the driverless pipeline — cone detection, SLAM, planning,
control, and the Mission Control UI — is a separate job involving
Docker. It is covered in [SETUP.md](SETUP.md) from step 3 onward, and
the architecture is in [AUTONOMY.md](AUTONOMY.md).
