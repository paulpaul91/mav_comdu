Header header

# Current status of a uav :
# HOVERING: try to be hovered
# HOVERED: already hovered, i.e., stabled
# PATHPOLLOWING: Following a path
# LOADING: try to load at some place
# LOADED:  already loaded
# RESTING: resting at some place 

int8  uav_status

int8  HOVERING      = 0
int8  PATHFOLLOWING = 1
int8  LOADING       = 2
int8  HOVERED       = 3
int8  LOADED        = 4
int8  RESTING       = 5
