obviously
=========

Open Robotic Vision and Utilities Library
You can install all neccessary packages by using the installObviously script
Type "sudo ./installObviously" to the terminal. 

Initial Checkout
================
Use EGit extension to enable Eclipse integration (recommended)

 or use the command line
 
git clone https://github.com/stefanmay/obviously.git

There are project files for Eclipse. Please update the GIT index in order to ignore workstation specific changes:

git update-index --assume-unchanged .project

git update-index --assume-unchanged .cproject

Configuration of GIT client

git config --global user.name "Your Name Here"

git config --global user.email "<acronym>@ohm"

git config --global credential.helper cache
