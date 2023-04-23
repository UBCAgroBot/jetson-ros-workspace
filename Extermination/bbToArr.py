def BBtoArr(weed_id, weed_position, currentSpeed, timeDelay):
    
    sprayerArray = "000000000"

    # find which sprayer column each side of the box is in
    side1Column = 8 - (weed_position[2] // (1 / 9))
    side2Column = 8 - (weed_position[4] // (1 / 9))

    # turn 0 to 1 in the columns that contain the bounding box
    if side1Column != side2Column:
        for i in range(side1Column, side2Column):
            sprayerArray[i] = 1

    else:
        sprayerArray[side1Column]

    # calculate time until the middle of the bounding box reaches the bottom of the frame
    timingForShoot = TopToBottomDistance * (weed_position[1] + weed_position[5])/(2 * currentSpeed) - timeDelay

    return (weed_id, sprayerArray, timingForShoot)