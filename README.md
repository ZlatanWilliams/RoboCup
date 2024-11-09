# RoboCup --- Robotics TDP Team 8
## File Structure (Temporarily)
Here is the basic structure of the codes:
```
│   README.md
├─detectors                          # store the detectors (may be unnecessary)
└─webots_simulation                  # main simulation codes
    ├─choregraphe_convert_motion     # self-generated motion files uploaded by Yichu
    ├─controllers                    # store all the controllers
    │  ├─nao_soccer                  # previous controller
    │  ├─OD_2                        # new controller with new ball detector created by Abood 
    │  └─original_controller         # the backup of the original controller
    ├─motions                        # store all the motion files
    │      Backwards.motion
    │      Forwards.motion
    │      ...
    └─worlds                         # webots world files
```
Note that no model weights (i.e. *.pt files) are stored in this repository because it is too big. The model weights are stored in Microsoft Teams under the folder `model_weights`. Please download them and place them in the right folder. You can upload new model weights to Microsoft Teams if you have trained a better one.
## Codes Update Policy & Iteration Management
We use this repository to record and update our project codes. This repository is linked to our _Linear_ group, so every pull request of this repository will be tracked in _Linear_. Here are the rules for contributing to this project:
1. **DO NOT DIRECTLY EDIT THE MAIN BRANCH.** This is very important for any group code work. Here are the basic procedures to start your contribution:
```bash
# make sure you are in your code workspace, use the git clone command to fetch the newest version of the codes
git clone https://github.com/ZlatanWilliams/RoboCup.git   # you may need to log in since this is a private repository
cd RoboCup                                                # enter this repository
git checkout -b <new_branch_name>                         # create a new branch and start coding, you can name it whatever you like
git branch                                                # use this command to check which branch you are in currently
```
2. After modifying, you can upload your codes. Here are the basic commands:
```bash
git status                                                # check current status, you may see some lines in red listing the changes you have made
git add --all                                             # add all the changes to this commit
git add <file_path>                                       # if you don't want to add all the files, you can add certain files by this command
git status                                                # now you can use this command again to double-check, you may find the red lines become green,
                                                          # which means they are added to this commit
git commit -m '(the description of this commit)'          # to commit the changes, you MUST add descriptions to claim why you made these changes
git push                                                  # finally push the commit to the remote repository, you may see some errors at the first commit
                                                          # of a new branch, just follow the instructions in the terminal to fix it
```
3. Now you can see your commit on the GitHub website. If you and other group members have confirmed that your branch is worthy of merging, you can start the pull requests. But, before that, you need to check Linear. Usually, every modification of this repository is based on the issues created in Linear. Every issue has a code in the form of `ROB-xx`.
