# message define target's state (position and velocity) and its covariance 
Header header          # Headers for the message and sequence ID with consecutively increasing ID
float64[6] state       # Target state: position and velocity 
float64[6] cov_row1        # row 1 of covariance matrix
float64[6] cov_row2        # row 2 of covariance matrix
float64[6] cov_row3        # row 3 of covariance matrix
float64[6] cov_row4        # row 4 of covariance matrix
float64[6] cov_row5        # row 5 of covariance matrix
float64[6] cov_row6        # row 6 of covariance matrix
int8 Veh_ID                # Vehicle ID, from 1-255  
bool ekf_enable 
bool dekf_enable 
float64 time 




