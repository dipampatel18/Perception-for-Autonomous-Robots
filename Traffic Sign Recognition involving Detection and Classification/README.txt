Steps for Execution of Project 4: Traffic Sign Recognition

Instructions for running the code-

1. Extract the folder ‘dipu18_proj4’ to any of your folders.

2. ‘dipu18_proj4’ contains the following files-

	I. P4_Submission
		
		a) Code
			1. Sample
			2. vlfeat-0.9.21-bin
			3. Classifier.m
			4. Classifier.mat
			5. Recognizer.m

		b) Output
			1. Traffic Sign Detection.mp4
		c) README.txt
		d) Report.pdf

3. Now in the Code folder, create a folder named 'Input Frames' and
move all the 2861 frames inside this folder from the Dataset.

3. Next, open the following folders-
	
	vlfeat-0.9.21-bin -> vlfeat-0.9.21 -> toolbox

4. Open the 'vl_setup.m' file and execute it in MATLAB

5. Thereafter, run the 'vl_version' code in the command window
and it should display 0.9.21 as its current version. 

6. Now, the Classifier.mat file has already been trained using the Classifier.m file.

7. Hence, directly execute the Recognizer.m file and it will start showing frames sequentially.
Once the program execution is completed, the video would be saved in the Output folder.