Header header
#Time
float64 time

#A priori estimate (12 states)
float64[12] priori_estimate

#Estimated state (12 states)
float64[12] state_estimate

#Covariance matrix (flattened 12x12 matrix)
float64[144] P

#Yaw from UGPS
float64 yaw_ugps
