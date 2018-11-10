from math import fabs, sin, acos, sqrt

def omegaFromRot(rot):
    alpha = (rot[0][0] + rot[1][1] + rot[2][2] - 1.0)/2.0
    if fabs(alpha - 1.0) < 1.0e-6:
        return [0.0, 0.0, 0.0]
    else:
        th = acos(alpha)
        s = sin(th)
    
        if s < sys.float_info.epsilon:
            return [ sqrt( (rot[0][0]+1)*0.5 ) * th,
                     sqrt( (rot[1][1]+1)*0.5 ) * th,
                     sqrt( (rot[2][2]+1)*0.5 ) * th ]
        k = -0.5 * th / s
        return [ (rot[1][2] - rot[2][1]) * k,
                 (rot[2][0] - rot[0][2]) * k,
                 (rot[0][1] - rot[1][0]) * k,
                ]
