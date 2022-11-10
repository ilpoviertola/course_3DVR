% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 1061.404914250427055 ; 1065.475160133910322 ];

%-- Principal point:
cc = [ 937.523957121473472 ; 525.969174509161689 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.033077302249744 ; -0.030804740900271 ; -0.000891267386646 ; -0.001218669802205 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 0.891781185878114 ; 0.849974680702428 ];

%-- Principal point uncertainty:
cc_error = [ 1.435593420469401 ; 1.062616771893487 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.001865788296517 ; 0.003066358195324 ; 0.000348753961175 ; 0.000510284936504 ; 0.000000000000000 ];

%-- Image size:
nx = 1920;
ny = 1080;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 26;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -2.185939e+00 ; -2.192122e+00 ; -1.245740e-01 ];
Tc_1  = [ -5.493002e+01 ; -6.355543e+01 ; 3.883824e+02 ];
omc_error_1 = [ 1.367433e-03 ; 1.700681e-03 ; 3.325126e-03 ];
Tc_error_1  = [ 5.327378e-01 ; 3.934612e-01 ; 4.812142e-01 ];

%-- Image #2:
omc_2 = [ 1.838687e+00 ; 1.876468e+00 ; 7.091268e-01 ];
Tc_2  = [ -3.060770e+01 ; -4.648365e+01 ; 3.707044e+02 ];
omc_error_2 = [ 1.347918e-03 ; 1.091315e-03 ; 2.054272e-03 ];
Tc_error_2  = [ 5.094334e-01 ; 3.743554e-01 ; 4.608946e-01 ];

%-- Image #3:
omc_3 = [ -1.959751e+00 ; -2.178129e+00 ; -9.239174e-01 ];
Tc_3  = [ -3.104571e+00 ; -7.011995e+01 ; 2.407063e+02 ];
omc_error_3 = [ 6.664727e-04 ; 1.348290e-03 ; 2.107136e-03 ];
Tc_error_3  = [ 3.371594e-01 ; 2.463550e-01 ; 3.061334e-01 ];

%-- Image #4:
omc_4 = [ -1.648413e+00 ; -1.989349e+00 ; 1.688865e-01 ];
Tc_4  = [ -9.397167e+01 ; -1.043271e+02 ; 3.189623e+02 ];
omc_error_4 = [ 9.315026e-04 ; 1.244139e-03 ; 1.736319e-03 ];
Tc_error_4  = [ 4.395848e-01 ; 3.233391e-01 ; 3.157544e-01 ];

%-- Image #5:
omc_5 = [ -1.828424e+00 ; -1.744573e+00 ; -6.197687e-01 ];
Tc_5  = [ -1.126675e+02 ; -4.671667e+01 ; 2.354806e+02 ];
omc_error_5 = [ 7.677762e-04 ; 1.144166e-03 ; 1.765580e-03 ];
Tc_error_5  = [ 3.257132e-01 ; 2.474599e-01 ; 3.076721e-01 ];

%-- Image #6:
omc_6 = [ 1.835552e+00 ; 1.927422e+00 ; -6.104929e-01 ];
Tc_6  = [ -6.000900e+01 ; -7.163681e+01 ; 4.394852e+02 ];
omc_error_6 = [ 1.063899e-03 ; 1.301397e-03 ; 2.131316e-03 ];
Tc_error_6  = [ 5.963117e-01 ; 4.382808e-01 ; 3.831330e-01 ];

%-- Image #7:
omc_7 = [ -1.641717e+00 ; -2.447584e+00 ; 8.657707e-01 ];
Tc_7  = [ -6.743678e-01 ; -1.123593e+02 ; 4.390610e+02 ];
omc_error_7 = [ 1.322354e-03 ; 1.278880e-03 ; 2.308208e-03 ];
Tc_error_7  = [ 6.002489e-01 ; 4.371483e-01 ; 3.294494e-01 ];

%-- Image #8:
omc_8 = [ 1.883683e+00 ; 1.413534e+00 ; -1.330165e-01 ];
Tc_8  = [ -8.430483e+01 ; -5.619341e+01 ; 3.932466e+02 ];
omc_error_8 = [ 1.105576e-03 ; 1.168725e-03 ; 1.784045e-03 ];
Tc_error_8  = [ 5.370880e-01 ; 3.945750e-01 ; 4.273801e-01 ];

%-- Image #9:
omc_9 = [ -2.188838e+00 ; -2.226597e+00 ; -4.314845e-02 ];
Tc_9  = [ -4.049832e+01 ; -8.297883e+01 ; 3.902531e+02 ];
omc_error_9 = [ 1.407987e-03 ; 1.702090e-03 ; 3.342166e-03 ];
Tc_error_9  = [ 5.356411e-01 ; 3.932701e-01 ; 4.712249e-01 ];

%-- Image #10:
omc_10 = [ -1.614942e+00 ; -1.959746e+00 ; 5.868554e-01 ];
Tc_10  = [ 6.010106e+00 ; -1.123794e+02 ; 4.846413e+02 ];
omc_error_10 = [ 1.216851e-03 ; 1.533057e-03 ; 2.219593e-03 ];
Tc_error_10  = [ 6.659684e-01 ; 4.815509e-01 ; 3.853320e-01 ];

%-- Image #11:
omc_11 = [ 1.894397e+00 ; 1.785719e+00 ; 5.245039e-01 ];
Tc_11  = [ -1.656731e+01 ; -6.856760e+01 ; 3.626555e+02 ];
omc_error_11 = [ 1.264871e-03 ; 1.095464e-03 ; 1.993707e-03 ];
Tc_error_11  = [ 4.987532e-01 ; 3.632581e-01 ; 4.310234e-01 ];

%-- Image #12:
omc_12 = [ 2.123351e+00 ; 2.029298e+00 ; 8.423477e-01 ];
Tc_12  = [ 4.684961e+01 ; -4.977404e+01 ; 3.214947e+02 ];
omc_error_12 = [ 1.511578e-03 ; 9.552882e-04 ; 2.121716e-03 ];
Tc_error_12  = [ 4.502820e-01 ; 3.265042e-01 ; 4.039678e-01 ];

%-- Image #13:
omc_13 = [ -1.767680e+00 ; -1.791762e+00 ; -1.501146e-02 ];
Tc_13  = [ -8.902028e+01 ; -7.068977e+01 ; 3.613727e+02 ];
omc_error_13 = [ 9.394713e-04 ; 1.317604e-03 ; 1.891738e-03 ];
Tc_error_13  = [ 4.934503e-01 ; 3.651802e-01 ; 3.542796e-01 ];

%-- Image #14:
omc_14 = [ -1.903292e+00 ; -2.287667e+00 ; 7.391109e-01 ];
Tc_14  = [ -5.043430e+01 ; -9.583890e+01 ; 4.719533e+02 ];
omc_error_14 = [ 1.466725e-03 ; 1.393883e-03 ; 2.691820e-03 ];
Tc_error_14  = [ 6.435861e-01 ; 4.702275e-01 ; 3.830350e-01 ];

%-- Image #15:
omc_15 = [ 1.955071e+00 ; 1.867264e+00 ; -3.442381e-01 ];
Tc_15  = [ -6.119399e+01 ; -4.034948e+01 ; 4.074049e+02 ];
omc_error_15 = [ 1.245656e-03 ; 1.242016e-03 ; 2.311378e-03 ];
Tc_error_15  = [ 5.520752e-01 ; 4.067980e-01 ; 3.985708e-01 ];

%-- Image #16:
omc_16 = [ 1.787678e+00 ; 1.652546e+00 ; 2.438938e-01 ];
Tc_16  = [ -7.940056e+01 ; -9.489999e+01 ; 3.634823e+02 ];
omc_error_16 = [ 1.093289e-03 ; 1.220204e-03 ; 1.800690e-03 ];
Tc_error_16  = [ 5.054057e-01 ; 3.660830e-01 ; 4.176934e-01 ];

%-- Image #17:
omc_17 = [ 2.164950e+00 ; 1.768990e+00 ; 7.982192e-01 ];
Tc_17  = [ -1.169520e+02 ; -5.772706e+01 ; 2.737661e+02 ];
omc_error_17 = [ 1.297191e-03 ; 9.303486e-04 ; 2.183032e-03 ];
Tc_error_17  = [ 4.043794e-01 ; 2.894326e-01 ; 3.757255e-01 ];

%-- Image #18:
omc_18 = [ -1.539927e+00 ; -1.920466e+00 ; 2.993404e-01 ];
Tc_18  = [ -1.651110e+01 ; -1.234840e+02 ; 3.879482e+02 ];
omc_error_18 = [ 9.825450e-04 ; 1.419567e-03 ; 1.821331e-03 ];
Tc_error_18  = [ 5.360380e-01 ; 3.885201e-01 ; 3.308086e-01 ];

%-- Image #19:
omc_19 = [ 1.643603e+00 ; 2.298499e+00 ; -5.183542e-01 ];
Tc_19  = [ 7.225331e+01 ; -5.868511e+01 ; 4.556027e+02 ];
omc_error_19 = [ 1.326515e-03 ; 1.340824e-03 ; 2.710345e-03 ];
Tc_error_19  = [ 6.251155e-01 ; 4.613872e-01 ; 4.263810e-01 ];

%-- Image #20:
omc_20 = [ 1.986895e+00 ; 1.631139e+00 ; -1.459879e-01 ];
Tc_20  = [ -1.895498e+02 ; -2.695042e+01 ; 3.841464e+02 ];
omc_error_20 = [ 1.236326e-03 ; 1.469384e-03 ; 2.160040e-03 ];
Tc_error_20  = [ 5.359067e-01 ; 4.063836e-01 ; 4.810626e-01 ];

%-- Image #21:
omc_21 = [ -2.106672e+00 ; -2.133416e+00 ; -4.573310e-01 ];
Tc_21  = [ -7.458417e+01 ; -8.246297e+01 ; 2.362928e+02 ];
omc_error_21 = [ 8.075365e-04 ; 1.137051e-03 ; 2.122550e-03 ];
Tc_error_21  = [ 3.364245e-01 ; 2.476968e-01 ; 2.869878e-01 ];

%-- Image #22:
omc_22 = [ -1.628687e+00 ; -2.109328e+00 ; 6.213821e-01 ];
Tc_22  = [ -1.894945e+01 ; -9.718690e+01 ; 4.190434e+02 ];
omc_error_22 = [ 1.123665e-03 ; 1.442757e-03 ; 2.075624e-03 ];
Tc_error_22  = [ 5.707525e-01 ; 4.143524e-01 ; 3.270653e-01 ];

%-- Image #23:
omc_23 = [ -1.869789e+00 ; -2.020149e+00 ; 4.177090e-01 ];
Tc_23  = [ -8.934400e+01 ; -8.299675e+01 ; 3.450996e+02 ];
omc_error_23 = [ 1.071521e-03 ; 1.195518e-03 ; 1.943772e-03 ];
Tc_error_23  = [ 4.704671e-01 ; 3.441328e-01 ; 3.156439e-01 ];

%-- Image #24:
omc_24 = [ -1.804064e+00 ; -1.756529e+00 ; -6.855571e-01 ];
Tc_24  = [ -1.145850e+02 ; -9.897841e+01 ; 3.063666e+02 ];
omc_error_24 = [ 9.471606e-04 ; 1.198890e-03 ; 1.978290e-03 ];
Tc_error_24  = [ 4.322871e-01 ; 3.300074e-01 ; 3.940311e-01 ];

%-- Image #25:
omc_25 = [ -1.692434e+00 ; -1.913592e+00 ; 1.585442e-01 ];
Tc_25  = [ -3.396228e+01 ; -1.247280e+02 ; 3.723419e+02 ];
omc_error_25 = [ 1.041485e-03 ; 1.438591e-03 ; 2.030532e-03 ];
Tc_error_25  = [ 5.173004e-01 ; 3.774709e-01 ; 3.487640e-01 ];

%-- Image #26:
omc_26 = [ -1.850669e+00 ; -2.346094e+00 ; 5.808439e-01 ];
Tc_26  = [ -5.921106e-01 ; -6.972821e+01 ; 4.872744e+02 ];
omc_error_26 = [ 1.477088e-03 ; 2.082595e-03 ; 3.727148e-03 ];
Tc_error_26  = [ 6.630095e-01 ; 4.842821e-01 ; 4.536424e-01 ];

