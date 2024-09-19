define laser ranger(

  sensor(
    range [ 0.0  10.0 ]
    fov 240.0
    samples 240
  )

  color "black"
  size [ 0.04 0.12 0.1 ]

  block( points 4
     point[0] [0 0]
     point[1] [0 1]
     point[2] [1 1]
     point[3] [1 0]
     z [0.5 0.75]
  ) 
  
)


define simple_robot position(

  odom_error [0.03 0.03 0.00 0.05]
  localization "odom"
  size [ 0.4 0.2 0.2 ]
  gui_nose 1
  drive "diff"

  block( points 6
        point[0] [-0.1 0.15]
        point[1] [-0.1 -0.15]
        point[2] [-0.17 -0.15]
        point[3] [-0.2 -0.12]
        point[4] [-0.2 0.12]
        point[5] [-0.17 0.15]
        z [0.5 1]
     color "light grey"
  )

  block( points 4
        point[0] [-0.1 0.15]
        point[1] [0.1 0.15]
        point[2] [0.1 -0.15]
        point[3] [-0.1 -0.15]
        z [1 0.5]
     color "light grey"
  ) 

  block( points 6
        point[0] [0.1 0.15]
        point[1] [0.27 0.15]
        point[2] [0.3 0.12]
        point[3] [0.3 -0.12]
        point[4] [0.27 -0.15]
        point[5] [0.1 -0.15]
        z [0.5 1]
     color "light grey"
  )

  block( points 4
        point[0] [0.07 0.15]
        point[1] [0.07 -0.15]
        point[2] [-0.07 -0.15]
        point[3] [-0.07 0.15]
        z [1.5 1.6]
     color "grey"
  )

  block( points 4
        point[0] [0.08 0.145]
        point[1] [0.08 0.115]
        point[2] [0.06 0.115]
        point[3] [0.06 0.145]
        z [1.2 1.5]
     color "grey"
  )

  block( points 4
        point[0] [0.1 0.15]
        point[1] [0.1 0.13]
        point[2] [0.08 0.13]
        point[3] [0.08 0.15]
        z [1 1.3]
     color "grey"
  )

  block( points 4
        point[0] [0.08 -0.145]
        point[1] [0.08 -0.115]
        point[2] [0.06 -0.115]
        point[3] [0.06 -0.145]
        z [1.2 1.5]
     color "grey"
  )

  block( points 4
        point[0] [0.1 -0.15]
        point[1] [0.1 -0.13]
        point[2] [0.08 -0.13]
        point[3] [0.08 -0.15]
        z [1 1.3]
     color "grey"
  )

  block( points 4
        point[0] [-0.06 0.145]
        point[1] [-0.06 0.115]
        point[2] [-0.08 0.115]
        point[3] [-0.08 0.145]
        z [1.2 1.5]
     color "grey"
  )

  block( points 4
        point[0] [-0.08 0.15]
        point[1] [-0.08 0.13]
        point[2] [-0.1 0.13]
        point[3] [-0.1 0.15]
        z [1 1.3]
     color "grey"
  )

  block( points 4
        point[0] [-0.06 -0.145]
        point[1] [-0.06 -0.115]
        point[2] [-0.08 -0.115]
        point[3] [-0.08 -0.145]
        z [1.2 1.5]
     color "grey"
  )

  block( points 4
        point[0] [-0.08 -0.15]
        point[1] [-0.08 -0.13]
        point[2] [-0.1 -0.13]
        point[3] [-0.1 -0.15]
        z [1 1.3]
     color "grey"
  )

  laser(pose [ 0.20 0.0 -0.2 0.0 ])
)
