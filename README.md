<h1 align="center">
  2026 Robot

  <p align="center">
      <a href="https://github.com/recordrobotics/2026-robot/actions/workflows/ci.yml"><img alt="CI" src="https://github.com/recordrobotics/2026-robot/actions/workflows/ci.yml/badge.svg?branch=main"></a>
      <a href="https://docs.recordrobotics.org/"><img alt="Read the Docs" src="https://img.shields.io/readthedocs/2024-control?logo=readthedocs&labelColor=%23556bc2"></a>
      <a href="https://github.com/recordrobotics/2026-robot/actions/workflows/ci.yml"><img alt="Unit Tests Status" src="https://img.shields.io/github/check-runs/recordrobotics/2026-robot/main?nameFilter=JUnit%20Test%20Report&logo=gradle&label=tests&labelColor=purple"></a>
      <a href="https://sonarcloud.io/summary/new_code?id=recordrobotics-2026-robot"><img alt="Quality Gate Status" src="https://sonarcloud.io/api/project_badges/measure?project=recordrobotics-2026-robot&metric=alert_status"></a>
      <a href="https://sonarcloud.io/summary/new_code?id=recordrobotics-2026-robot"><img alt="Coverage" src="https://sonarcloud.io/api/project_badges/measure?project=recordrobotics-2026-robot&metric=coverage"></a>
  </p>
</h1>

<img src="assets/Microwave Control Scheme.png">

-----------------------------------------------

Link to the [CAD](https://cad.onshape.com/documents/626ac996a2def381c67777a7/w/0394bc9a0f6401474961ac58/e/e4e09c8e63856bfaeddd637f).

Here are our [goals](https://recordrobotics.notion.site/1714851f43d58095ac37c44f40ad3b70?v=1714851f43d5800d9462000c01589958) for the season.

Docs at [docs.recordrobotics.org](https://docs.recordrobotics.org/)

-----------------------------------------------

## Extensions

This repository recommends the following VS Code extensions for the best development experience:

<table>
  <tr>
    <td align="center">
      <a href="https://marketplace.visualstudio.com/items?itemName=streetsidesoftware.code-spell-checker">
        <img src="https://streetsidesoftware.gallerycdn.vsassets.io/extensions/streetsidesoftware/code-spell-checker/4.2.3/1753028947698/Microsoft.VisualStudio.Services.Icons.Default" width="64" alt="Code Spell Checker"/><br/>
        <b>Code Spell Checker</b>
      </a>
    </td>
    <td align="center">
      <a href="https://marketplace.visualstudio.com/items?itemName=SonarSource.sonarlint-vscode">
        <img src="https://sonarsource.gallerycdn.vsassets.io/extensions/sonarsource/sonarlint-vscode/4.29.0/1755515927519/Microsoft.VisualStudio.Services.Icons.Default" width="64" alt="SonarQube"/><br/>
        <b>SonarQube</b>
      </a>
    </td>
    <td align="center">
      <a href="https://marketplace.visualstudio.com/items?itemName=shengchen.vscode-checkstyle">
        <img src="https://shengchen.gallerycdn.vsassets.io/extensions/shengchen/vscode-checkstyle/1.4.2/1680054146028/Microsoft.VisualStudio.Services.Icons.Default" width="64" alt="Checkstyle for Java"/><br/>
        <b>Checkstyle for Java</b>
      </a>
    </td>
    <td align="center">
      <a href="https://marketplace.visualstudio.com/items?itemName=edonet.vscode-command-runner">
        <img src="https://edonet.gallerycdn.vsassets.io/extensions/edonet/vscode-command-runner/0.0.124/1680934084102/Microsoft.VisualStudio.Services.Icons.Default" width="64" alt="Command Runner"/><br/>
        <b>Command Runner</b>
      </a>
    </td>
  </tr>
</table>

### SonarQube Setup

To use the custom SonarQube rules configured for this repository, you need to copy the SonarQube rules from [`.vscode/settings.json`](.vscode/settings.json) into your **user settings**:

1. Open the Command Palette (`Ctrl+Shift+P` or `Cmd+Shift+P` on Mac).
2. Type and select **Preferences: Open User Settings (JSON)**.
3. Copy the `"sonarqube.rules"` section from [`.vscode/settings.json`](.vscode/settings.json) into your user settings file.

This ensures SonarQube uses the same code quality rules as the repository.

### How to use Command Runner

This repository is set up to show useful commands in a menu when pressing `Ctrl+Shift+R`

<img width="1244" height="160" alt="image" src="https://github.com/user-attachments/assets/bbc890e2-5ade-4d3b-a0a5-5e3c6b319964" />

#### Available commands

- **tuning swerve encoders** — when selecting this command an additional window prompting for the type of module appears. The options are `kraken|falcon` with default `kraken`

  <img width="996" height="141" alt="image" src="https://github.com/user-attachments/assets/5431c4f5-0d50-4350-90ad-8c85f21207a4" />

#### Keybindings

An example keybinding is provided in [`.vscode/keybindings.json`](.vscode/keybindings.json), allowing you to set custom keybinds to certain commands.

**IMPORTANT:** to add keybinds:

1. Open the Command Palette (`Ctrl+Shift+P` or `Cmd+Shift+P` on Mac).
2. Type and select **Preferences: Open Keyboard Shortcuts (JSON)**.
3. This is where you can define the custom keybinds (**this applies for your whole vscode user not just this repo**)

-----------------------------------------------

## Git LFS

This repository uses [Git Large File Storage](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-git-large-file-storage) to store large files like the robot model assets for AdvantageScope.

### Setting up Git LFS

First, make sure Git LFS is installed by downloading it from https://git-lfs.com/

Then run this command
```bash
git lfs install
```

**Note:** if the command exits with the error:

```
Hook already exists: pre-push
```

rerun the command with the `--force` argument and re-add the spotless pre-push hook with `./gradlew spotlessInstallGitPrePushHook`.

```bash
git lfs install --force
./gradlew spotlessInstallGitPrePushHook
```

### Storing files using Git LFS

The types of files that are stored using Git LFS are determined by the [`.gitattributes`](.gitattributes) file.

The Git LFS extension automatically handles downloads and uploads behind the scenes and appears the same as a normal file in the commits.

To add a new file type to Git LFS storage run

```bash
git lfs track "*.glb" # replace with target type
```

This command adds an entry in the [`.gitattributes`](.gitattributes) file and updates the metadata for existing files matching the new filter. This means you will need to commit both the [`.gitattributes`](.gitattributes) and matching file changes.


## Spotless Pre-Push Hook

Run this command to set up a git hook that will format your code with spotless before you push: `./gradlew spotlessInstallGitPrePushHook`.
This is a per-repository hook and needs to be run on every clone.

### Push fails in VSCode UI but works from terminal with `git push`

This usually means the `JAVA_HOME` environment variable is not set when VSCode calls the pre-push hook which in turn runs `./gradlew` with the wrong Java.

To fix add the following line in [`.git/hooks/pre-push`](.git/hooks/pre-push)

```diff
 ##### SPOTLESS HOOK START #####
+export JAVA_HOME='C:\Users\Public\wpilib\2026\jdk'
 SPOTLESS_EXECUTOR=<path_to_repo>/gradlew.bat
 if ! $SPOTLESS_EXECUTOR spotlessCheck ; then
```

## Git Config

Before starting make sure to setup git with these recommended settings and aliases:

- `git config --global pull.rebase true`
  - This ensures your local commits will be rebased on top of any incoming changes when pulling
- `git config --global alias.origin 'reset --hard origin/$(git branch --show-current)'`
  - This is an alias which allows you to use `git origin` to reset your current branch to origin
- `git config --global alias.fpush 'push --force-with-lease --force-if-includes'`
  - This is an alias which allows you to use `git fpush` for safe force pushing
- `git config --global alias.reb '!f() { git checkout main && git pull && git checkout "$1" && git pull && git rebase --onto main "$(git merge-base main "$1")" "$1"; }; f'`
  - This is an alias which allows you to use `git reb [branch name]` which rebases this branch on to main. Run from anywhere, it will automatically take you to the branch and pull them. Remember to fpush the branch afterwards.
- `git config --global alias.reball '!git for-each-ref --format="%(refname:short)" refs/heads/ | xargs -n1 git reb'`
  - This is an alias which allows you to use `git reball` to rebase all branches to main. This uses reb, so add it too. Run from anywhere, it will automatically take you to the branches and pull them. Remember to fpush all branches afterwards
- `git config --global alias.logline "log --graph --pretty=format:'%Cred%h%Creset -%C(yellow)%d%Creset %s %Cgreen(%cr) %C(bold blue)<%an>%Creset' --abbrev-commit"`
  - This is an alias which which allows you to use `git logline` to show a log of past commits to the current branch with useful information. Press q to quit

-------------------------------------------------

[Record Robotics](https://www.recordrobotics.org/)
