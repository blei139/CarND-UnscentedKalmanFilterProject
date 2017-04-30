# CarND-UnscentedKalmanFilterProject

Instructions on how to run the unscented kalman filter project:
------------------------------------------------------------------------------------------------------------------------------------------
1) make a directory called build with mkdir build and cd build to go to the build directory
2) use cmake and make to create an executable file called UnscentedKF.exe with the following command:
cmake .. && make
3) run the input data files:
 ./UnscentedKF ../data/obj_pose-laser-radar-synthetic-input.txt output_synth.txt > logsynth;
 ./UnscentedKF ../data/sample-laser-radar-measurement-data-1.txt output1.txt > log1;
 ./UnscentedKF ../data/sample-laser-radar-measurement-data-2.txt output2.txt > log2;
 4) open up each log file to see the results:
 more logsynth;
 more log1;
 more log2;
 
Instructions for comparison and debugging purpose:
------------------------------------------------------------------------------------------------------------------------------------------
In the output directory,  just look at the log and output files for comparison and debugging purpose:
for example,
sdiff logsynth [your own logsyth file] | more;
sdiff log1 [your own log1 file] | more;
sdiff log2 [your own log2 file] | more;

Instructions for verification of the actual ouput log files with the expected log files in a C++ testbench:
------------------------------------------------------------------------------------------------------------------------------------------
In the testbench directory, use my C++ executable file to compare the actual output log files with the expected log files.
The excutable C++ testbench file will compare two float strings and print out the test results just like design verification in chip design.
Here are some examples:
./logFileComparison "../build/logsynth" "logsynth_gold" > logsynthResult; 
more logsynthResult;
./logFileComparison "../build/log1" "log1_gold" > log1Result;
more log1Result;
./logFileComparison "../build/log2" "log2_gold" > log2Result;
more log2Result;
