# mpc_local_planner
Integrates mpc_lib with move_base 

# how it works

TODO


# Screenshots:

Modified (bending) recursive shadow casting to get useful obstacle boundaries in circular order:

```bash
38 outline points.
'@' is the robot.
'.' is empty space.
'=' is obstacle in the map.
'#' is an outline point.
. . . . . . . . . . = = = = = = = = = .
. . . . . . . . . V = = = = = = = = = .
. . . . . . . . . U = = = = = = = = = .
. . . . . . . . . T = = = = = = = = = .
. . . . . . . . . . S R Q = = = = = = .
. . . . . . . . . . . . P O N = = = . .
. . . . . . . . . . . . . . M L = . . .
. . . . . . . . . . . . . . . K J . . .
. . . . . . . . . . . . . . . . . . . .
. . . W X . . . . . . . . . . . . . . .
. . = = Y Z . . . . @ . . . . I = = . .
. . = = = [ . . . . . . . . . H = = = .
. . = = = \ . . . . . . . . F G = = . .
. . = = = ] . . . . . . . . E = = = . .
. . . = ^ . . . . . . . . . . D = . . .
. . . . . . . ` a . . . . . . . . . . .
. . . . . . _ = b . . . . . . . . . . .
. . . . . . = = c . f A B . . . . . . .
. . . . . . . . . . e = C . . . . . . .
. . . . . . . . . . d = = . . . . . . .
```

Path and obstacle poly fitting:  
![Path and obstacle poly example](https://user-images.githubusercontent.com/6284428/87867471-96cf1b00-c9aa-11ea-96c9-c6f3652dea62.png)
