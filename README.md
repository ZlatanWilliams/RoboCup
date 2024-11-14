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
We use this repository to record and update our project codes. This repository is linked to our _Linear_ group, so every pull requests (PR) of this repository will be tracked in _Linear_. Here are the rules for contributing to this project:
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
3. Now you can see your commit on the GitHub website. If you and other group members have confirmed that your branch is worthy of merging, you can start the PR. But, before that, you need to check the issues on Linear. Usually, every modification of this repository is based on the issues created in Linear. Every issue has a code in the form of `ROB-xx`. For example, `ROB-16` is an issue named "Arrange current codes". If you want to move forward with this issue, here are the steps you can take：
    1. Click `Pull requests` in the top menu bar, and click the green button `New pull request`.
    2. You may see a hint in a big yellow box. Click the green button `Create pull request`.
    3. Modify the title or description of your PR in the form of `magic word + issue ID`. For example, `close ROB-16`, `part of ROB-16` or `fixes ROB-16`. For more information please refer to [official documents](https://linear.app/docs/github?tabs=206cad22125a).
    4. Submit your PR. You can see in Linear that the activity of issue ROB-16 is automatically updated.
    5. Merge your PR to the main branch. This can be done by any member of the group.
4. We always keep the main branch as the latest version of our codes. When you find your branches are behind the main branch, do the following steps:
```bash
git checkout main                    # change current branch to the main branch
git pull                             # fetch the latest codes of the main branch
git checkout <your_branch_name>      # change current branch to your branch
git merge main                       # merge the codes in the main branch to your branch
```
> These steps could work when there are no conflicts between your branch with the main branch. If conflicts occur, solve them first.
5. You can delete your branch after you merge all the modifications to the main branch.
