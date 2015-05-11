# CPR
Cascade pose regression 
###################################################################
#                                                                 #
#    Cascaded Pose Regression Toolbox                             #
#    http://vision.ucsd.edu/~pdollar/research.html                #
#    Piotr Dollar (pdollar-at-caltech.edu)                        #
#                                                                 #
###################################################################

1. Introduction.

If using the Cascaded Pose Regression Toolbox, please cite the following work in any resulting publication:

@inproceedings{ DollarCVPR10pose,
  author = "P. Doll\'ar and P. Welinder and P. Perona",
  title = "Cascaded Pose Regression",
  booktitle = "CVPR",
  year = "2010",
}

###################################################################

2. License.

This code is published under the Simplified BSD License.
Please read bsd.txt for more info.

###################################################################

3. Installation.

This code is written for Matlab (tested with versions 20010b-2012a) and requires the Matlab Image Processing Toolbox. Additionally, Piotr's Matlab Toolbox (version 3.00 or later) is also required. It can be downloaded at: 
  http://vision.ucsd.edu/~pdollar/toolbox/doc/index.html
Make sure the random ferns code in the toolbox is working (see and run the example code in classify/fernsRegTrain.m).

###################################################################

4. Getting Started.

The code is quite compact, as is the method. Start with cprDemo.m which first generates toy data, then trains a CPR model, and finally displays the error and example results. The demo should take under 5 minutes to run, including training time. To run CPR with your own data start with cprDemo.m but replace the toy data with real data (using the same format). You may also want to update poseGt.m to define a new pose model (models in poseGt are provided for fish, mouse, and face data) and poseLabeler to label ground truth pose.

###################################################################

5. Contents.

Code:
   cprApply     - Apply multi stage pose regressor.
   cprDemo      - Demo demonstrating CPR code using toy data.
   cprTrain     - Train multistage pose regressor.
   poseGt       - Object pose annotations struct.
   poseLabeler  - Pose labeler for static images.

Other:
    bsd.txt     - Simplified BSD License.
    readme.txt  - This file.

###################################################################

6. History / ToDo.

Version 1.0 (08/06/2012)
 - initial version

###################################################################
