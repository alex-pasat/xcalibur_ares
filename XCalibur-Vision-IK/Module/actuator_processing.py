def velocity_ratios(q):
    ratio = [[0]* 4 for _ in range(len(q)-1)] # only need ratios of other quantities

    for j in range(len(ratio)):
        ratio[j][0] = ((q[j+1][0]-q[j][0])/(q[j+1][2]-q[j][2]))
        ratio[j][1] = ((q[j+1][1]-q[j][1])/(q[j+1][2]-q[j][2]))
        ratio[j][2] = ((q[j+1][3]-q[j][3])/(q[j+1][2]-q[j][2]))
        ratio[j][3] = ((q[j+1][4]-q[j][4])/(q[j+1][2]-q[j][2]))
    
    return ratio
