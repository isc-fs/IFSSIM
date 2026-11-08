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

## Updating the sim

Settings, UPDATES. The sim checks the repo's GitHub Releases when it starts
and says on the main menu when a newer build is out.

- **Builds**: **Stable** (the default) gets the tagged releases; **Latest**
  gets the last build of `dev`, newer but less tested.
- **Update and restart** downloads it, restarts the sim into it and keeps the
  build you had next to it (`<folder>-previous`): **Back to the previous
  build** returns to it. A `settings.json` you changed in the sim's folder is
  kept as `settings.previous.json`, and tracks you added there come along.
- It signs in to GitHub with the login git keeps from getting the team's
  code; **Check now** signs in if it can't.
- After updating, **Rebuild pipeline** (below it), so the stack is built from
  code that matches the new sim.

Builds are published with `tools/publish_windows_build.sh stable|latest`
(Git Bash, UE 5.8, `gh` logged in), until a CI runner does it.

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
| docker compose failed: ... | `docker compose logs dv_pipeline_stack` (or `mission_control_backend`) in the repo folder. |
| The backend didn't see the sim within 2 minutes | `docker compose logs mission_control_backend`; check nothing else uses ports 8000 or 41451. |

A Spanish (or any other language) Windows is fine: the sim asks Windows where
Documents is rather than guessing its name, and reads git's and Docker's
own messages, not Windows' translated ones. Paths with spaces or accents are
fine too.
