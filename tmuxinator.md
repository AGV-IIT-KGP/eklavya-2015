# tmuxinator
###Executing commands sequentially in different panes and windows

Open terminal

$ sudo apt-get install tmux

$ sudo apt-get install ruby

$ sudo su

$ gem install tmuxinator

Exit from su

The directory .tmuxinator contains the config file.

$ export EDITOR='gedit'

$ echo $EDITOR

Ignore if already included in ~/.bashrc

###To start new session

$ tmux new-session

Result:A green strip must appear at bottom

###To create a new project session:

$ mux new `project_name`

Result:config file opens in default editor 
###Writing the config file

A default config file will look like this:

```
# ~/.tmuxinator/sample.yml

name: sample
root: ~/

# Optional. tmux socket
# socket_name: foo

# Runs before everything. Use it to start daemons etc.
# pre: sudo /etc/rc.d/mysqld start

# Runs in each window and pane before window/pane specific commands. Useful for setting up interpreter versions.
# pre_window: rbenv shell 2.0.0-p247

# Pass command line options to tmux. Useful for specifying a different tmux.conf.
# tmux_options: -f ~/.tmux.mac.conf

# Change the command to call tmux.  This can be used by derivatives/wrappers like byobu.
# tmux_command: byobu

# Specifies (by name or index) which window will be selected on project startup. If not set, the first window is used.
# startup_window: logs

windows:
  - editor:
      layout: main-vertical
      panes:
        - vim
        - guard
  - server: bundle exec rails s
  - logs: tail -f log/development.log
```

Change this to :

```
# ~/.tmuxinator/project.yml

name: project
root: ~/

# Optional tmux socket
# socket_name: foo

# Runs before everything. Use it to start daemons etc.
# pre: sudo /etc/rc.d/mysqld start

# Runs in each window and pane before window/pane specific commands. Useful for setting up interpreter versions.
# pre_window: rbenv shell 2.0.0-p247

# Pass command line options to tmux. Useful for specifying a different tmux.conf.
# tmux_options: -f ~/.tmux.mac.conf

# Change the command to call tmux.  This can be used by derivatives/wrappers like byobu.
# tmux_command: byobu

# Specifies (by name or index) which window will be selected on project startup. If not set, the first window is used.
# startup_window: logs

# Controls whether the tmux session should be attached to automatically. Defaults to true.
# attach: false

# Runs after everything. Use it to attach to tmux with custom options etc.
# post: tmux -CC attach -t project

windows:
  - controls:
      layout: tiled
      panes:
        - sudo ./xboxdrv.sh
        - sleep 1;sudo ./launch.sh
        - sleep 5;cd arduino && sudo ./arduino
        - sleep 12;sudo ./robo_launch.sh
        - sleep 10;sudo ./encoder_pkg.sh
  - planner:
      layout: tiled
      panes:
        - #empty
        - #empty
        - #empty
        - #empty
  - sensors:
      layout: tiled
      panes:
        - #empty
        - #empty
        - #empty
        - #empty
# - server: bundle exec rails s
# - logs: tail -f log/development.log
```

Here,Controls,planner,sensors are the name of the windows under which sub-windows termed as `panes` open up.

Each window can be designated for carrying out a particular process and each pane to carry out a particular command.

Layout decides the splitting size and shape of the panes opening up.Generally,`tiled` will be preferd.

Below the text `panes:` ,you can add as many commands in format shown above.And each command will be executed in a new pane only

A `#empty` command specifies opening a new pane without any initial command executed.

Similiarly , a new window can also be contructed by editing the above config file in controls,planner,sensors format.

#####WARNING
The config file is space sensitive.Use of `TAB` is not recommended.
If any command is initilaized with sudo ,and proper changes haven't been made to `sudoers` file,then that particular command will not executed simultaneosly as others (It will prompt for password).

Its better to make the following changes to `sudoers` file so as to increase the time limit and password versatility in any other terminal.

To open sudoers file:
Type `$ visudo` in a new terminal.

```
Defaults        env_reset,timestamp_timeout=120
Defaults        mail_badpass
Defaults        secure_path="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:$
Defaults        !tty_tickets
```
Since,tmux executes all the commands in the panes simultaneosly,it can give errors for the processes that must be executed sequentially.Under such case, use of command `sleep [time in seconds]` will be preferable as shown in controls window of the above config file.

Moreover,a new session can also be executed separetely for planner and sensors modules of the above config file incase they have prerequsite commands like mentioned in controls module.

###For ease in navigation during the tmux session,

Open a new terminal

`$ sublime .tmux.conf `

Write the following lines in this empty conf file and save:

```
set-option -g mouse-select-pane on
set -g mouse-resize-pane on
set -g mouse-select-pane on
set -g mouse-select-window on
```
Here,all the Configuration process ends.

###Execution commands in Tmux session

Open a new terminal.

`$ tmux new-session`

`$ mux project_name`
Result:All windows with corresponding panes starts executing the commands.

To kill all execution at once,

type in any pane of the the session as:

`$ killall tmux`

To edit the config file of `project_name`:
During the session,

`$ tmux open project_name`



