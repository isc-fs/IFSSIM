# From the build zip to an autonomous run

For team members who got the sim as a zip (a GitHub Release, or from a
teammate) and want the pipeline to drive it. You don't need Unreal Engine or
Visual Studio for this; for working on the sim itself, see
[SETUP.md](SETUP.md).

Windows only for now. On macOS and Linux, clone the repo yourself and follow
[SETUP.md](SETUP.md).

## What you install yourself

**Docker Desktop**: <https://www.docker.com/products/docker-desktop/>. It
needs admin rights, and the first time it may ask to install WSL and reboot.
Once it's installed, open it, then:

- Settings, Resources, Network: tick **Enable host networking**, then
  **Apply & restart**. Without it the pipeline runs but the sim can't reach
  it (the sim tells you if it's off).

That's all. You also need a GitHub account in the **isc-fs** organization: the
repos are private.

## What the sim does

Unzip the build anywhere, run `IFSSIM.exe`, pick an event and a track, pick
**Autonomous** and press **START**. The first time, the sim:

1. **Asks for the team's code** (the IFSSIM repo, with the pipeline in it as a
   submodule), with four answers:
   - **Get it**: clones it into `Documents\IFSSIM-repo` (`Documents\IFSSIM` is where the
     sim keeps its session logs). If your Documents folder
     is in OneDrive, it uses `C:\Users\<you>\IFSSIM-repo` instead: OneDrive would
     try to sync gigabytes of build output and lock files under git and Docker.
   - **Somewhere else...**: you pick the folder.
   - **I have it...**: you already cloned it; pick its folder (the one with
     `docker-compose.yml`).
   - **Not now**.
2. **Installs git** if it isn't there (with winget: Windows asks for
   permission once).
3. **Clones the repo** (`git clone --recursive --branch dev`). Your browser
   opens to sign in to GitHub; git remembers it after that. The Unreal assets
   (Git LFS) are skipped: running the stack doesn't need them.
4. **Opens Docker Desktop** if it isn't running.
5. **Builds the pipeline and the backend images** from that code: 5 to 10
   minutes the first time, then never again until you rebuild them.
6. **Starts them**, waits until the backend sees the sim, and starts the run.

Each step shows under START. After the first time, START just starts the
stack (if it isn't running) and the run.

The folder is remembered (Settings, PIPELINE CODE, where **Change...** points
it somewhere else).

## Working on the pipeline

The pipeline is `pipeline/` in that folder: a git submodule (the
IFS09-DV-PIPELINE repo), checked out at a commit rather than a branch. Make
one before changing anything:

```bash
cd ~/Documents/IFSSIM-repo/pipeline
git switch -c my-change
```

Then:

- **Python:** edit, press **START**. START copies your pipeline code into the
  container before every run, and the mission restarts the nodes, so they run
  your edit. No terminal needed.
- **C++, messages, launch files, a `setup.py`:** Settings, PIPELINE CODE,
  **Rebuild pipeline**, then START. It builds the pipeline and the backend
  from your code (seconds when little changed, minutes otherwise) and
  restarts them.

Your work goes in a PR to the pipeline repo; once merged, a PR here moves the
submodule to it.

## Watching the pipeline

All in the sim, no web page or terminal:

- **Pipeline view.** During an autonomous run the sim draws, over the real
  track, what the pipeline believes: this scan's detected cones (magenta),
  the SLAM cone map (white rings), where SLAM puts the car (pink arrow), the
  planned path (green) and the cone chains it follows (blue, yellow). The map
  sliding off the real cones is SLAM drift. Esc, **Pipeline view**, turns it
  on or off.
- **Logs.** The Logs page shows what the pipeline's nodes (or the backend)
  print, with a filter and **Problems only**; **Save to a file** for a
  teammate.
- **Bags.** Pick **Record a bag** on the Drive page before START. The Bags
  page lists the recordings; **Replay in the sim** puts the car back on the
  track and moves it as it drove while the pipeline view shows what the
  pipeline saw at each moment. Esc stops it.

## Updating the sim

Settings, UPDATES. The sim checks the repo's GitHub Releases when it starts
and says on the main menu when a newer build is out.

- **Builds**: **Stable** (the default) gets the tagged releases; **Latest**
  gets the last build of `dev`, newer but less tested.
- **Update and restart** downloads it, restarts the sim into it and keeps the
  build you had next to it (`<folder>-previous`): **Back to the previous
  build** returns to it. A `settings.json` you changed in the sim's folder is
  kept as `settings.previous.json`, and tracks you added there come along.
- The sim compares the team's code with the commit it was built from
  (Settings, PIPELINE CODE). When your copy is older, has no changes and is
  on `dev`, **Bring the stack in step** moves it forward and rebuilds. The
  pipeline moves with it only when you haven't touched it: a branch of your
  own or any change in `pipeline/` stays exactly as it is. Otherwise it only
  warns, and nothing is moved.

### Publishing a build

Every release has Linux: the bench PC builds it on each `v*` tag and merge
to `dev`, and creates the release. Windows and macOS are added by whoever
has the sim on that OS, with write access and UE 5.8:

```bash
tools/publish_build.sh latest   # the last build of dev, on `latest-dev`
tools/publish_build.sh stable   # a release: v<ProjectVersion>
```

From a clean tree, with `gh` logged in (Git Bash on Windows).

- **A new release:** bump `ProjectVersion` in `Config/DefaultGame.ini`
  above the newest release, merge it to `dev`, then tag it
  (`git tag v1.2.0 && git push origin v1.2.0`). The bench PC creates the
  release with Linux. Then `git checkout v1.2.0` and
  `tools/publish_build.sh stable` adds your OS. If the bench PC is down,
  `--first` creates the release from your OS.
- **Latest:** from `dev`, up to date. The script only moves `latest-dev`
  forward, so publishing from an older commit does nothing.

The script refuses another commit than the tag's and a version older than
the newest release (Stable would go back), and skips an OS already there. Each zip
goes up with a `.sha256`. GitHub takes up to 2 GiB per file, with no limit on
a release's total or on downloads.

### A machine that builds on its own (optional)

A runner builds on every `v*` tag (`.github/workflows/package-*.yml`, which
call the same script). Register it for packaging only, with its own label
and none of the defaults, so no other job lands on it:

```
config.cmd --url https://github.com/isc-fs/IFSSIM --token <token from an admin> ^
  --name <you>-win --labels ifssim-package-windows --no-default-labels
```

Start it with `run.cmd` when the machine can build; while it's off, the job
waits and anyone can publish by hand. Labels: `ifssim-package-linux` (the
bench PC; runs first), `ifssim-package-windows`, `ifssim-package-mac` (after
Linux).

The repo is public, and a fork's pull request runs the fork's own workflow
file, which can ask for any label. Settings, Actions, "Require approval for
all external contributors" stays on, and a fork's run that changes
`.github/workflows` is never approved.

## Keeping the code up to date

The sim doesn't pull for you. To get the team's latest code:

```bash
cd ~/Documents/IFSSIM-repo     # or wherever it is
git pull --recurse-submodules
```

then **Rebuild pipeline** in Settings.

## When something goes wrong

The message under START says what failed and what to do. The usual ones:

| Message | What to do |
|---|---|
| GitHub said no: the account can't see isc-fs/IFSSIM | Ask to be added to the isc-fs organization, or sign in with that account (Windows Credential Manager, remove `git:https://github.com`, then START again). |
| Docker isn't installed | Install Docker Desktop (above), open it once, START again. |
| Docker's host networking is off | Docker Desktop, Settings, Resources, Network, Enable host networking, Apply & restart. |
| docker compose failed: ... | The Logs page (Pipeline or Backend) says why. |
| The backend didn't see the sim within 2 minutes | The Logs page, Backend; check nothing else uses ports 8000 or 41451. |

A Spanish (or any other language) Windows is fine: the sim asks Windows where
Documents is rather than guessing its name, and reads git's and Docker's
own messages, not Windows' translated ones. Paths with spaces or accents are
fine too.
