tmux new-session -d -s robot        # I named it robot because i felt like it

tmux setw -g mouse on               # This is so you can click between panes

# Split the session into a small bottom pane and split top horizontally
tmux split-window -d -p 10\; split-window -h\; split-window -d\; split-window -d;

tmux attach-session -t robot