% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 363.914069148211297 ; 365.113816157794247 ];

%-- Principal point:
cc = [ 256.757574036044218 ; 207.313133949448854 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.077040890433597 ; -0.176205313884485 ; -0.000911858559963 ; -0.000440486659380 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 2.030816088811866 ; 1.955868237484777 ];

%-- Principal point uncertainty:
cc_error = [ 2.751142797533326 ; 2.267292141151186 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.013647006594920 ; 0.027760262037117 ; 0.002156149300400 ; 0.002694794436714 ; 0.000000000000000 ];

%-- Image size:
nx = 512;
ny = 424;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 27;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -2.185037e+00 ; -2.200192e+00 ; -1.130585e-01 ];
Tc_1  = [ -1.066805e+02 ; -6.484379e+01 ; 3.896129e+02 ];
omc_error_1 = [ 1.058851e-02 ; 1.158273e-02 ; 2.309415e-02 ];
Tc_error_1  = [ 3.002672e+00 ; 2.493066e+00 ; 3.236450e+00 ];

%-- Image #2:
omc_2 = [ 1.830662e+00 ; 1.875572e+00 ; 7.110291e-01 ];
Tc_2  = [ -8.206069e+01 ; -4.779010e+01 ; 3.698477e+02 ];
omc_error_2 = [ 8.590689e-03 ; 7.571325e-03 ; 1.379684e-02 ];
Tc_error_2  = [ 2.873153e+00 ; 2.356634e+00 ; 3.172465e+00 ];

%-- Image #3:
omc_3 = [ -1.954371e+00 ; -2.183782e+00 ; -9.210625e-01 ];
Tc_3  = [ -5.475290e+01 ; -7.122961e+01 ; 2.408585e+02 ];
omc_error_3 = [ 5.259484e-03 ; 8.242068e-03 ; 1.272716e-02 ];
Tc_error_3  = [ 1.899489e+00 ; 1.573717e+00 ; 2.063729e+00 ];

%-- Image #4:
omc_4 = [ -1.641764e+00 ; -1.987371e+00 ; 1.756299e-01 ];
Tc_4  = [ -1.455694e+02 ; -1.055333e+02 ; 3.194711e+02 ];
omc_error_4 = [ 6.095628e-03 ; 6.915605e-03 ; 9.959042e-03 ];
Tc_error_4  = [ 2.433187e+00 ; 2.062320e+00 ; 2.177692e+00 ];

%-- Image #5:
omc_5 = [ -1.824035e+00 ; -1.748719e+00 ; -6.169723e-01 ];
Tc_5  = [ -1.641684e+02 ; -4.806975e+01 ; 2.358086e+02 ];
omc_error_5 = [ 5.638177e-03 ; 6.760159e-03 ; 1.027981e-02 ];
Tc_error_5  = [ 1.812751e+00 ; 1.632551e+00 ; 2.175586e+00 ];

%-- Image #6:
omc_6 = [ 1.829904e+00 ; 1.933823e+00 ; -6.110226e-01 ];
Tc_6  = [ -1.113438e+02 ; -7.297650e+01 ; 4.396342e+02 ];
omc_error_6 = [ 6.571579e-03 ; 8.322046e-03 ; 1.350816e-02 ];
Tc_error_6  = [ 3.312273e+00 ; 2.768861e+00 ; 2.581673e+00 ];

%-- Image #7:
omc_7 = [ -1.636139e+00 ; -2.454195e+00 ; 8.648165e-01 ];
Tc_7  = [ -5.191858e+01 ; -1.134590e+02 ; 4.385883e+02 ];
omc_error_7 = [ 7.633074e-03 ; 7.465768e-03 ; 1.421398e-02 ];
Tc_error_7  = [ 3.333385e+00 ; 2.732189e+00 ; 2.230161e+00 ];

%-- Image #8:
omc_8 = [ 1.876322e+00 ; 1.418012e+00 ; -1.340066e-01 ];
Tc_8  = [ -1.357122e+02 ; -5.765948e+01 ; 3.927898e+02 ];
omc_error_8 = [ 6.879387e-03 ; 7.491995e-03 ; 1.148602e-02 ];
Tc_error_8  = [ 2.989513e+00 ; 2.500534e+00 ; 2.914851e+00 ];

%-- Image #9:
omc_9 = [ -2.183269e+00 ; -2.232978e+00 ; -3.548413e-02 ];
Tc_9  = [ -9.195329e+01 ; -8.424706e+01 ; 3.908139e+02 ];
omc_error_9 = [ 1.072336e-02 ; 1.165612e-02 ; 2.369886e-02 ];
Tc_error_9  = [ 3.006189e+00 ; 2.482048e+00 ; 3.173447e+00 ];

%-- Image #10:
omc_10 = [ -1.606569e+00 ; -1.960148e+00 ; 5.870932e-01 ];
Tc_10  = [ -4.496095e+01 ; -1.134113e+02 ; 4.844508e+02 ];
omc_error_10 = [ 7.270270e-03 ; 8.532938e-03 ; 1.287835e-02 ];
Tc_error_10  = [ 3.700226e+00 ; 2.999528e+00 ; 2.632002e+00 ];

%-- Image #11:
omc_11 = [ 1.890523e+00 ; 1.792481e+00 ; 5.406851e-01 ];
Tc_11  = [ -6.742577e+01 ; -6.957494e+01 ; 3.606751e+02 ];
omc_error_11 = [ 7.959587e-03 ; 7.347102e-03 ; 1.339127e-02 ];
Tc_error_11  = [ 2.788686e+00 ; 2.268985e+00 ; 2.963319e+00 ];

%-- Image #12:
omc_12 = [ 2.120060e+00 ; 2.037078e+00 ; 8.415253e-01 ];
Tc_12  = [ -4.958692e+00 ; -5.037816e+01 ; 3.220186e+02 ];
omc_error_12 = [ 9.394045e-03 ; 6.348190e-03 ; 1.360583e-02 ];
Tc_error_12  = [ 2.496986e+00 ; 2.034180e+00 ; 2.728837e+00 ];

%-- Image #13:
omc_13 = [ -1.761957e+00 ; -1.797237e+00 ; -6.808005e-03 ];
Tc_13  = [ -1.401741e+02 ; -7.234438e+01 ; 3.622082e+02 ];
omc_error_13 = [ 6.221483e-03 ; 7.521389e-03 ; 1.068808e-02 ];
Tc_error_13  = [ 2.729730e+00 ; 2.320014e+00 ; 2.409676e+00 ];

%-- Image #14:
omc_14 = [ -1.888716e+00 ; -2.283461e+00 ; 7.377629e-01 ];
Tc_14  = [ -1.013391e+02 ; -9.756330e+01 ; 4.725151e+02 ];
omc_error_14 = [ 8.477470e-03 ; 7.896255e-03 ; 1.590494e-02 ];
Tc_error_14  = [ 3.574633e+00 ; 2.963018e+00 ; 2.604488e+00 ];

%-- Image #15:
omc_15 = [ 1.949786e+00 ; 1.871879e+00 ; -3.388502e-01 ];
Tc_15  = [ -1.128645e+02 ; -4.165830e+01 ; 4.065375e+02 ];
omc_error_15 = [ 8.018989e-03 ; 8.548212e-03 ; 1.545384e-02 ];
Tc_error_15  = [ 3.067623e+00 ; 2.563405e+00 ; 2.696443e+00 ];

%-- Image #16:
omc_16 = [ 1.784540e+00 ; 1.653222e+00 ; 2.448490e-01 ];
Tc_16  = [ -1.311120e+02 ; -9.615804e+01 ; 3.632393e+02 ];
omc_error_16 = [ 6.866842e-03 ; 8.076907e-03 ; 1.223214e-02 ];
Tc_error_16  = [ 2.827478e+00 ; 2.320353e+00 ; 2.875680e+00 ];

%-- Image #17:
omc_17 = [ 2.161688e+00 ; 1.774102e+00 ; 7.950488e-01 ];
Tc_17  = [ -1.687588e+02 ; -5.924504e+01 ; 2.742499e+02 ];
omc_error_17 = [ 8.459257e-03 ; 7.033044e-03 ; 1.573233e-02 ];
Tc_error_17  = [ 2.334880e+00 ; 1.905055e+00 ; 2.679856e+00 ];

%-- Image #18:
omc_18 = [ -1.531668e+00 ; -1.920495e+00 ; 2.973078e-01 ];
Tc_18  = [ -6.792328e+01 ; -1.245239e+02 ; 3.874894e+02 ];
omc_error_18 = [ 6.129963e-03 ; 7.886741e-03 ; 1.058684e-02 ];
Tc_error_18  = [ 2.961975e+00 ; 2.409815e+00 ; 2.250483e+00 ];

%-- Image #19:
omc_19 = [ 1.638788e+00 ; 2.303820e+00 ; -5.146194e-01 ];
Tc_19  = [ 2.056714e+01 ; -5.946785e+01 ; 4.552131e+02 ];
omc_error_19 = [ 8.685643e-03 ; 8.937528e-03 ; 2.059214e-02 ];
Tc_error_19  = [ 3.465773e+00 ; 2.861302e+00 ; 2.821144e+00 ];

%-- Image #20:
omc_20 = [ 1.986008e+00 ; 1.639393e+00 ; -1.426081e-01 ];
Tc_20  = [ -2.412383e+02 ; -2.887404e+01 ; 3.844642e+02 ];
omc_error_20 = [ 8.020611e-03 ; 1.016861e-02 ; 1.492125e-02 ];
Tc_error_20  = [ 2.969665e+00 ; 2.652201e+00 ; 3.267725e+00 ];

%-- Image #21:
omc_21 = [ 2.088896e+00 ; 1.655365e+00 ; 9.941023e-02 ];
Tc_21  = [ -2.572392e+02 ; -3.566462e+01 ; 3.676675e+02 ];
omc_error_21 = [ 1.185728e-02 ; 1.408335e-02 ; 2.156371e-02 ];
Tc_error_21  = [ 2.929621e+00 ; 2.621287e+00 ; 3.593771e+00 ];

%-- Image #22:
omc_22 = [ -1.625122e+00 ; -2.109999e+00 ; 6.156102e-01 ];
Tc_22  = [ -7.099715e+01 ; -9.802802e+01 ; 4.183189e+02 ];
omc_error_22 = [ 6.709280e-03 ; 7.880641e-03 ; 1.203824e-02 ];
Tc_error_22  = [ 3.155478e+00 ; 2.592936e+00 ; 2.224020e+00 ];

%-- Image #23:
omc_23 = [ -1.866556e+00 ; -2.025401e+00 ; 4.191511e-01 ];
Tc_23  = [ -1.407911e+02 ; -8.408809e+01 ; 3.451097e+02 ];
omc_error_23 = [ 6.723245e-03 ; 6.532797e-03 ; 1.118704e-02 ];
Tc_error_23  = [ 2.599314e+00 ; 2.193783e+00 ; 2.143459e+00 ];

%-- Image #24:
omc_24 = [ -1.798194e+00 ; -1.758811e+00 ; -6.844059e-01 ];
Tc_24  = [ -1.660590e+02 ; -1.002593e+02 ; 3.064390e+02 ];
omc_error_24 = [ 6.704203e-03 ; 7.348774e-03 ; 1.194866e-02 ];
Tc_error_24  = [ 2.406969e+00 ; 2.120597e+00 ; 2.651492e+00 ];

%-- Image #25:
omc_25 = [ -1.684684e+00 ; -1.916190e+00 ; 1.616810e-01 ];
Tc_25  = [ -8.516084e+01 ; -1.259279e+02 ; 3.723323e+02 ];
omc_error_25 = [ 6.557008e-03 ; 8.030597e-03 ; 1.170929e-02 ];
Tc_error_25  = [ 2.859066e+00 ; 2.342940e+00 ; 2.320673e+00 ];

%-- Image #26:
omc_26 = [ -1.844161e+00 ; -2.348655e+00 ; 5.751275e-01 ];
Tc_26  = [ -5.219253e+01 ; -7.090247e+01 ; 4.867524e+02 ];
omc_error_26 = [ 8.898096e-03 ; 1.223474e-02 ; 2.235205e-02 ];
Tc_error_26  = [ 3.681721e+00 ; 3.023506e+00 ; 2.914010e+00 ];

%-- Image #27:
omc_27 = [ -2.097180e+00 ; -2.133697e+00 ; -4.535397e-01 ];
Tc_27  = [ -1.264836e+02 ; -8.343063e+01 ; 2.366038e+02 ];
omc_error_27 = [ 6.243373e-03 ; 6.882104e-03 ; 1.318115e-02 ];
Tc_error_27  = [ 1.901120e+00 ; 1.618401e+00 ; 2.018478e+00 ];

