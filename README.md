# ROS 2 jetson workspace

This branch is for the code that will be deployed to the jetson board to run on the robot. It used ROS 2 Humble Hawksbill.
## Getting Started

### Requirements

**For Developers**:

- [Ubuntu 22.04](https://releases.ubuntu.com/22.04/) (You can use [your laptop](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview), a VM, or a cloud provider such as [AWS](https://github.com/UBCAgroBot/jetson-ros-workspace/blob/main/docs/aws_developer_env.md))
- [GitHub SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) for you laptop/workspace

### Installation

Clone the repo and checkout to the ROS 2 branch with the commands below:

```bash
git clone --recurse-submodules git@github.com:UBCAgroBot/jetson-ros-workspace.git
git checkout ros2-workspace
```

To install `ros2`, follow [this page from the ROS 2 docs](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

Also, **make sure to complete** [Beginner CLI Tools](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html#) and [Beginner Client Libraries](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html) since they cover important fundamentals of ROS 2 and also guide you with setting up your environment and the installation of `colcon` and `rqt`.

### Troubleshooting

- If you see a similar error when running `colcon build` error:

  - ```text
      /usr/lib/python3.10/site-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
    ```

  - run `pip install setuptools==58.2.0` ([source](https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/))

- If you see any import errors for the modules when using `PyCharm` or `VS Code`follow [this](https://answers.ros.org/question/382798/vscode-rospy-import-error/) to inform the location of the modules to your IDE.

## Contribution

Please contribute to the project according to GitHub contribution guidelines outlined below.

### GitHub Contribution Guidelines

Please create a new branch from the latest version of main branch for your contribution. Create one branch for each individual feature / fix. After you finish writing and testing your feature / fix, open a pull request to merge your branch with the main. Please write meaningful comments to your commits and pull requests.

#### How to Branch

```bash
git checkout main
```

```bash
git pull
```

```bash
git checkout -b <new_branch_name> main
```

#### Naming your branch

Please name your branch as `<subteam-initial>/<your-name>/<short-description-of-feature-or-fix>`

For `<subteam-initial>`, use `n` for navigation, `i` for image recognition, `e` for extermination, `c` for chassis.

#### How to open a pull request

- Open the branch page in GitHub.
- Use the `contribute` button to open a PR (pull request).
- Your code is ready to be reviewed and merged. Great work!