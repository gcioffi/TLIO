_This code is a supplementary material to the paper "TLIO: Tight Learned Inertial Odometry". To use the code here requires the user to generate its own dataset and retrain. For more information about the paper and the video materials, please refer to our [website](https://cathias.github.io/TLIO/)._


# Installation
Dependencies tree can be retrieved from `pyproject.toml`.
It is written for the poetry tool. 
All dependencies can thus be installed at once in a new virtual environment with:
```shell script
cd src
poetry install
```
Then the virtual environment is accessible with:
```shell script
poetry shell
```

Alternatively, the dependencies are also specified and can be installed through `requirements.txt`. First create a virtual environment with python3 interpreter, then run
``` 
pip install -r requirements.txt
```

Next commands should be run from this environment.

# Dataset
A dataset is needed in the format of hdf5 to run with this code. The dataset tree structure looks like this under root directory `Dataset`:
```
Dataset
├── test.txt
├── train.txt
├── val.txt
├── seq1
│   ├── atttitude.txt
│   ├── calib_state.txt
│   ├── evolving_state.txt
│   └── data.hdf5
├── seq22
│   ├── atttitude.txt
│   ├── calib_state.txt
│   ├── evolving_state.txt
│   └── data.hdf5
...
```

`data.hdf5` contains raw and calibrated IMU data and processed ground truth data. It is used for both the network and the filter. `calib_state.txt` contains calibration states from VIO and is used for filter initialization. `atttitude.txt` and `evolving_state.txt` are the outputs from AHRS attitude filter and VIO pose estimates. These are not used by the filter, but loaded for comparison / debug purposes.

The generation of `data.hdf5` is specified in `gen_fb_data.py`, which requires interpolated stamped IMU measurement files and time-aligned VIO states files. The user can generate his/her own dataset with a different procedure to obtain the same fields to be used for network training and filter inputs.

## File formats

### Dataset lists

`test.txt`, `train.txt` and `val.txt` are list files specifying the split with one sequence name per row in the testing, training and validation datasets respectively. The name should be the same of the sequence directory for example `seq1` and `seq22` as above.


### Used to generate `data.hdf5`
Timestamps (t) are in microseconds (us). Each row corresponds to data in a single timestamp. All data is delimited by commas.

- `my_timestamps_p.txt` VIO timestamps.
  - [t]
  - Note: single column, skipped first 20 frames
- `imu_measurements.txt` raw and calibrated IMU data
  - [t, acc_raw (3), acc_cal (3), gyr_raw (3), gyr_cal (3), has_vio] 
 - Note: calibration through VIO calibration states. The data has been interpolated evenly between images around 1000Hz. Every timestamp in my_timestamps_p.txt will have a corresponding timestamp in this file (has_vio==1). 
- `evolving_state.txt` ground truth (VIO) states at IMU rate.
  - [t, q_wxyz (4), p (3), v (3)]
  - Note: VIO state estimates with IMU integration. Timestamps are from raw IMU measurements.
- `calib_state.txt` VIO calibration states at image rate (used in `data_io.py`)
  - [t, acc_scale_inv (9), gyr_scale_inv (9), gyro_g_sense (9), b_acc (3), b_gyr (3)]
  - Note: Changing calibration states from VIO.
- `atttitude.txt` AHRS attitude from IMU
  - [t, qw, qx, qy, qz]

# Network training and evaluation

## For training or evaluation of one model

There are three different modes for the network part.`--mode` parameter defines the behaviour of `main_net.py`. Select between `train`, `test` and `eval`. \
`train`: training a network model with training and validation dataset. \
`test`: running an existing network model on testing dataset to obtain concatenated trajectories and metrics. \
`eval`: running an exising network model and save all statistics of data samples for network performance evaluation.

### 1. Training:

**Parameters:** 

`--root_dir`: dataset root directory. Each subfolder of root directory is a dataset. \
`--train_list`: directory of the txt file with a list of training datasets. It should contain name of subfolder in root. \
`--val_list`: directory of the txt file with a list of validation datasets.  \It should contain name of subfolder in root.\
`--out_dir`: training output directory, where `checkpoints` and `logs` folders will be created to store trained models and tensorboard logs respectively. A `parameters.json` file will also be saved.

**Example:** 
```shell script
python3 src/main_net.py \
--mode train \
--root_dir data/Dataset \
--train_list data/Dataset/train.txt \
--val_list data/Dataset/val.txt \
--out_dir train_outputs
```

### 2. Testing:

**Parameters:** 

`--test_list`: path of the txt file with a list of testing datasets. \
`--model_path`: path of the trained model to test with. \
`--out_dir`: testing output directory, where a folder for each dataset tested will be created containing estimated trajectory as `trajectory.txt` and plots if specified. `metrics.json` contains the statistics for each dataset. 

**Example:**
```
python3 src/main_net.py \
--mode test \
--root_dir data/Dataset \
--test_list data/Dataset/test.txt \
--model_path models/resnet/checkpoint_*.pt \
--out_dir test_outputs
```

### 3. Evaluation:

**Parameters:** 

`--out_dir`: evaluation pickle file output directory. \
`--sample_freq`: the frequency of network input data sample tested in Hz. \
`--out_name`: (optional) output pickle file name.

**Example:**
```shell script
python3 src/main_net.py \
--mode eval \
--root_dir data/Dataset \
--test_list data/Dataset/test.txt \
--model_path models/resnet/checkpoint_*.pt \
--out_dir eval_outputs \
--sample_freq 5 \
--out_name resnet.pkl
```
Please refer to `main_net.py` for a full list of parameters.

## For batch evaluation on multiple models

Batch scripts are under src/batch_analysis module. Execute batch scripts from the src folder.

### Testing:

Batch testing tests a list of datasets using multiple models and for each model save the trajectories, plots and metrics into a separate model folder. Output tree structure looks like this:
```
batch_test_outputs
├── model1
│   ├── seq1
│   │   ├── trajectory.txt
│   │   └── *.png
│   ├── seq2
...
│   └── metrics.json
├── model2
│   ├── seq1
...
│   └── metrics.json
...
```

Create an output directory and go to the src folder
```shell script
mkdir batch_test_outputs
cd src
```
Run batch tests. `--model_globbing` is the globbing pattern to find all models to test.
```shell script
python -m batch_runner.net_test_batch \
--root_dir ../data/Dataset \
--data_list ../data/Dataset/test.txt \
--model_globbing "../models/*/checkpoint_*.pt" \
--out_dir ../batch_test_outputs \
```
To save plots as well, change parameter `save_plot` to True in `main_net.py`.

### Evaluation:

Batch evaluation runs the eval mode for multiple models, with various perturbation settings. Different perturbations result in a separate pickle file under each model folder. Output tree structure:
```
net_eval_outputs
├── model1
│   ├── d-bias-0.0-0.025-grav-0.0.pkl
│   ├── d-bias-0.0-0.05-grav-0.0.pkl
│   ├── d-bias-0.0-0.075-grav-0.0.pkl
│   ├── d-bias-0.0-0.0-grav-0.0.pkl
│   ├── d-bias-0.0-0.0-grav-10.0.pkl
│   ├── d-bias-0.0-0.0-grav-2.0.pkl
│   ├── d-bias-0.0-0.0-grav-4.0.pkl
│   ├── d-bias-0.0-0.0-grav-6.0.pkl
│   ├── d-bias-0.0-0.0-grav-8.0.pkl
│   ├── d-bias-0.0-0.1-grav-0.0.pkl
│   ├── d-bias-0.1-0.0-grav-0.0.pkl
│   ├── d-bias-0.2-0.0-grav-0.0.pkl
│   ├── d-bias-0.3-0.0-grav-0.0.pkl
│   ├── d-bias-0.4-0.0-grav-0.0.pkl
│   └── d-bias-0.5-0.0-grav-0.0.pkl
├── model2
│   ├── d-bias-0.0-0.025-grav-0.0.pkl
│   ├── d-bias-0.0-0.05-grav-0.0.pkl
...
```

In the current script, the following perturbation values are used: \
Accelerometer bias perturbation range: [0, 0.1, 0.2, 0.3, 0.4, 0.5] (m/s^2) \
Gyroscope bias perturbation range: [0, 0.025, 0.05, 0.075, 0.1] (rad/s) \
Gravity direction perturbation range: [0, 0, 2, 4, 6, 8, 10] (degrees) \
These can be changed in the script `batch_runner/net_eval_batch.py`, and for each perturbation range a pkl file will be saved with the range in the filename.

Create an output directory and go to the src folder
```shell script
mkdir batch_eval_outputs
cd src
```
Run batch evaluation
```shell script
python -m batch_runner.net_eval_batch \
--root_dir ../data/Dataset \
--data_list ../data/Dataset/test.txt \
--model_globbing "../models/*/checkpoint_*.pt" \
--out_dir ../net_eval_outputs \
--sample_freq 5.0
```

## Running analysis and generating plots

After running testing and evaluation in batches, the statistics are saved in either `metrics.json` or the generated pickle files. To visualize the results and compare between models, we provide scripts that display the results in an interactive shell through iPython. The scripts are under `src/analysis` module.

To visualize network testing results from `metrics.json` including trajectory metrics and testing losses, go to `src` folder and run
```shell script
python -m analysis.display_json \
--glob_dataset "../batch_test_output/*/"
```
This will leave you in an interactive shell with a preloaded panda DataFrame `d`. You can use it to visualize all metrics with the following helper function:
```shell script
plot_all_stats_net(d)
```

To visualize evaluation results from pickle files, run
```shell script
python -m analysis.display_pickle \
--glob_pickle "../batch_eval_outputs/*/*.pkl"
```
This gives access to all the sample data 3D displacement gt and errors, sigmas, mse and likelihood losses, 2D norm and angle gt and errors, and mahalanobis distance based on the regressed covariance. To plot sigmas vs. errors for example, run
```shell script
plot_sigmas(d)
```

# Running EKF with network displacement estimates

## Running EKF with one network model
 
Use `src/main_filter.py` for running the filter and parsing parameters. The program supports running multiple datasets on one specified network model.

**Parameters:**

`--model_path`: path to saved model checkpoint file. \
`--model_param_path`: path to parameter json file for this model. \
`--out_dir`: filter output directory. This will include a `parameters.json` file with filter parameters, and a folder for each dataset containing the logged states, default to `not_vio_state.txt`. \
`--erase_old_log`: overwrite old log files. If set to `--no-erase_old_log`, the program would skip running on the datasets if the output file already exists in the output directory. \
`--save_as_npy`: convert the output txt file to npy file and append file extension (e.g. `not_vio_state.txt.npy`) to save space. \
`--initialize_with_offline_calib`: initialize with offline calibration of the IMU. If set to `--no-initialize_with_offline_calib` the initial IMU biases will be initialized to 0.

**Example:**
```shell script
python3 src/main_filter.py \
--root_dir data/Dataset \
--data_list data/Dataset/test.txt \
--model_path models/resnet/checkpoint_75.pt \
--model_param_path models/resnet/parameters.json \
--out_dir filter_outputs \
--erase_old_log \
--save_as_npy \
--initialize_with_offline_calib
```
Please refer to `main_filter.py` for a full list of parameters.

## Batch running filter on multiple models and parameters

Batch script `batch_runner/filter_batch` provides functionality to run the main file in batch settings. Go to `src` folder to run the module and you can set the parameters to test within the script (e.g. different update frequencies).

**Example:**
```shell script
python -m batch_runner.run_batch \
--root_dir ../data/Dataset \
--data_list ../data/Dataset/test.txt \
--model_globbing "../models/*/checkpoint_*.pt" \
--out_dir ../batch_filter_outputs
```

## Batch running metrics and plot generation

To generate plots of the states of the filter and to generate `metrics.json` file for both the filter and network concatenation approaches, batch run `plot_state.py` on the existing filter and network testing outputs.

**Parameters:**

`--runname_globbing`: globbing pattern for all the model names to plot. This pattern should match between filter and ronin and exist in both `--filter_dir` and `--ronin_dir`. \
`--no_make_plots`: not to save plots. If removed plots will be saved in the filter output folders for each trajectory.

**Example:**
```shell script
python -m batch_runner.plot_batch \
--root_dir ../data/Dataset \
--data_list ../data/Dataset/test.txt \
--runname_globbing "*" \
--filter_dir ../batch_filter_outputs \
--ronin_dir ../batch_test_outputs \
```

Up to now a `metrics.json` file will be added to each model folder, and the tree structure would look like this:
```
batch_filter_outputs
├── model1
│   ├── seq1
│   │   ├── *.png
│   │   ├── not_vio_state.txt.npy
│   │   └── vio_states.npy
│   ├── seq1
│   │   ├── *.png
│   │   ├── not_vio_state.txt.npy
│   │   └── vio_states.npy
...
│   ├── metrics.json
│   └── parameters.json
├── model2
...
```

To generate plots from the metrics:
```shell script
python -m analysis.display_json \
--glob_dataset "../batch_filter_outputs/*/"
```

# Implementation Details

### Training and Testing on Simulation Data

With a trajectory simulation being set in GAZEBO, it is possible to record a ROSBAG including the most important data, namely angular velocity, linear acceleration values and ground-truth. In the python scripts introduced below, using the GT from simulation, the raw data are derived, choosing the IMU biases from a uniform distirbution and noises on bias from a gaussian distribution. The standard deviation values are specified in the "params" folder.
Choosing different biases and noises, it is possible to generate a big number of trajectories which can then be used for training the network. 
The goal of this step is to create a dataset as large as possible for training, in order to see the behavior and the quality of the predictions when the input data to the network is the equivalent real trajectory. 

### Parameters

A folder called "params" is generated in the directory: TLIO/src. In this folder, there are two yaml files called "dataloader_params.yaml" and "dataloader_params_snaga.yaml". If a local version of the code is used, then it is necessary to adapt dataloader_params and use it. It specifies the bagfile with the trajectory to train, the recorded topics (IMU + Odometry), the number of trajectories generated by "load_racing_data_from_rosbag" and the output directory in which the dataset will be generated. Furthermore, in this config file there are also the bias values and the standard deviations for IMU and bias noises on acceleration and angular velocity.
If SNAGA is used, instead, it is better to use the prepared dataloader_params_snaga. 

### PYScript: load_racing_data_from_rosbag.py

"load_racing_data_from_rosbag" loads a rosbag of a specified trajectory and generates different versions of it perturbing biases and noises on both IMU and biases. The number of versions to be obtained is specified by "n_trajectories: 2" in the "dataloader_params.yaml". 
Angular velocity and acceleration GT (from simulation) are imported, whereas calibrated (GT + noise on the IMU) and raw (GT + bias + noise on the IMU and bias) angular velocity and acceleration are generated. 
At this point, the following txt files will be produced: 

- my_timestamps_p.txt
- imu_measurements.txt
- evolving_state.txt

**COMMAND TO LAUNCH (on SNAGA)**

python3 src/dataloader/load_racing_data_from_rosbag.py --config src/params/dataloader_params_snaga.yaml

**COMMAND TO LAUNCH (on LAPTOP)**

python3 src/dataloader/load_racing_data_from_rosbag.py --config src/params/dataloader_params.yaml



### PYScript: load_random_trajectories_from_rosbag.py

"load_random_trajectories_from_rosbag" loads a certain numnber of rosbags, containing different randomly generated trajectories. Furthermore, each trajectory generates other versions of it, with perturbed biases and noises on both IMU and biases. The number of versions for each trajecotry is specified by "n_trajectories: " in the "dataloader_params.yaml". 
Angular velocity and acceleration GT (from simulation) are imported, whereas calibrated (GT + noise on the IMU) and raw (GT + bias + noise on the IMU and bias) angular velocity and acceleration are generated. 
At this point, the following txt files will be produced: 

- my_timestamps_p.txt
- imu_measurements.txt
- evolving_state.txt

**COMMAND TO LAUNCH (on SNAGA)**

python3 src/dataloader/load_random_trajectories_from_rosbag.py --config src/params/dataloader_params_snaga.yaml

**COMMAND TO LAUNCH (on LAPTOP)**

python3 src/dataloader/load_random_trajectories_from_rosbag.py --config src/params/dataloader_params.yaml


### PYScript: gen_racing_data.py

Launching "gen_racing_data.py", it is possible to get the hdf5 needed for the training step and the train.txt, test.txt and val.txt files.
When launching this script, a data directory --data_dir should be specified: TLIO/data/Dataset. 

**COMMAND TO LAUNCH:**

python3 src/dataloader/gen_racing_data.py --data_dir data/Dataset/



### SNAGA: Training

Training is carried out in SNAGA. 

**COMMAND TO LAUNCH:**

python3 src/main_net.py --mode train --root_dir data/Dataset_cpc_1_65/ --train_list data/Dataset_cpc_1_65/train.txt --val_list data/Dataset_cpc_1_65/val.txt --out_dir results/cpc_1_65_StepSize_wt1/ --batch_size 100 --imu_freq 500 --window_time 0.8 --epochs 100 

python3 src/main_net.py --mode train --root_dir /data/scratch/aurora/TLIO/data/Dataset_Multiple_Traj --train_list /data/scratch/aurora/TLIO/data/Dataset_Multiple_Traj/train.txt --val_list /data/scratch/aurora/TLIO/data/Dataset_Multiple_Traj/val.txt --out_dir /data/scratch/aurora/TLIO/results/Dataset_Multiple_Traj --batch_size 128 --imu_freq 500 --window_time 0.5 --epochs 50 --continue_from results/Dataset_Multiple_Traj/checkpoints/checkpoint_3.pt | tee output.txt


python3 src/main_net.py --mode train --root_dir /home/rpg/Desktop/TLIO/data/Dataset_finto --train_list /home/rpg/Desktop/TLIO/data/Dataset_finto/train.txt --val_list /home/rpg/Desktop/TLIO/data/Dataset_finto/val.txt --out_dir /home/rpg/Desktop/TLIO/results/Dataset_finto --batch_size 128 --imu_freq 500 --window_time 0.5 --epochs 1

python3 src/main_net.py --mode train --root_dir /data/scratch/aurora/TLIO/data/Dataset_cpc_1_65 --train_list /data/scratch/aurora/TLIO/data/Dataset_cpc_1_65/train.txt --val_list /data/scratch/aurora/TLIO/data/Dataset_cpc_1_65/val.txt --out_dir results/cpc_1_65_StepSize_wt1/ --batch_size 100 --imu_freq 500 --window_time 0.8 --epochs 100 

python3 src/main_net.py --mode train --root_dir /data/scratch/aurora/TLIO/data/Dataset_Multiple_Traj_New/ --train_list /data/scratch/aurora/TLIO/data/Dataset_Multiple_Traj_New/train.txt --val_list /data/scratch/aurora/TLIO/data/Dataset_Multiple_Traj_New/val.txt --out_dir /data/scratch/aurora/TLIO/results/Dataset_Multiple_Traj_New/ --batch_size 128 --imu_freq 500 --window_time 0.5 --epochs 50 | tee new_train.txt

python3 src/main_net.py --mode train --root_dir /data/scratch/aurora/TLIO/data/Dataset_Multiple_Traj_New/ --train_list /data/scratch/aurora/TLIO/data/Dataset_Multiple_Traj_New/train.txt --val_list /data/scratch/aurora/TLIO/data/Dataset_Multiple_Traj_New/val.txt --out_dir /data/scratch/aurora/TLIO/results/Dataset_Multiple_Traj_New0_2/ --batch_size 128 --imu_freq 500 --window_time 0.2 --epochs 50 | tee new_train0_2.txt


python3 src/main_net.py --mode train --root_dir /data/scratch/aurora/TLIO/data/Circle_No_Noise/ --train_list /data/scratch/aurora/TLIO/data/Circle_No_Noise/train.txt --val_list /data/scratch/aurora/TLIO/data/Circle_No_Noise/val.txt --out_dir /data/scratch/aurora/TLIO/results/Circle_No_Noise_wt05_lr_e_05_MSE_moreValidation/ --batch_size 100 --imu_freq 500 --window_time 0.5 --epochs 800 --lr 2e-05


python3 src/main_net.py --mode train --root_dir /data/scratch/aurora/TLIO/data/OverfitCircle1000/ --train_list /data/scratch/aurora/TLIO/data/OverfitCircle1000/train.txt --val_list /data/scratch/aurora/TLIO/data/OverfitCircle1000/val.txt --out_dir /data/scratch/aurora/TLIO/results/OverfitCircle1000/ --batch_size 100 --imu_freq 500 --window_time 0.5 --epochs 800 --lr 1e-05


python3 src/main_net.py --mode train --root_dir /data/scratch/aurora/TLIO/data/Dataset_Multiple_Traj50/ --train_list /data/scratch/aurora/TLIO/data/Dataset_Multiple_Traj50/train.txt --val_list /data/scratch/aurora/TLIO/data/Dataset_Multiple_Traj50/val.txt --out_dir /data/scratch/aurora/TLIO/results/Random_wt05_lr_e_05_MSE/ --batch_size 128 --imu_freq 500 --window_time 0.5 --epochs 1000 --lr 1e-05


python3 src/main_net.py --mode train --root_dir /data/scratch/aurora/TLIO/data/CircleNoise50/ --train_list /data/scratch/aurora/TLIO/data/CircleNoise50/train.txt --val_list /data/scratch/aurora/TLIO/data/CircleNoise50/val.txt --out_dir /data/scratch/aurora/TLIO/results/CircleNoise50_wt05_lr_e_05_MSE/ --batch_size 128 --imu_freq 500 --window_time 0.5 --epochs 200--lr 1e-05


python3 src/main_net.py --mode train --root_dir /data/scratch/aurora/TLIO/data/Dataset_Multiple_Traj50/ --train_list /data/scratch/aurora/TLIO/data/Dataset_Multiple_Traj50/train.txt --val_list /data/scratch/aurora/TLIO/data/Dataset_Multiple_Traj50/val.txt --out_dir /data/scratch/aurora/TLIO/results/Random_wt05_lr_e_05_MSE/ --batch_size 128 --imu_freq 500 --window_time 0.5 --epochs 1000 --lr 1e-06 --continue_from results/Random_wt05_lr_e_05_MSE/checkpoints/checkpoint_396.pt


python3 src/main_net.py --mode train --root_dir /data/scratch/aurora/TLIO/data/Lemniscate_No_Noise/ --train_list /data/scratch/aurora/TLIO/data/Lemniscate_No_Noise/train.txt --val_list /data/scratch/aurora/TLIO/data/Lemniscate_No_Noise/val.txt --out_dir /data/scratch/aurora/TLIO/results/Lemniscate_No_Noise_wt05_lr_e_05_MSE_moreValidation/ --batch_size 128 --imu_freq 500 --window_time 0.5 --epochs 800 --lr 1e-06


### SNAGA: Testing

Testing is carried out in SNAGA. 

**COMMAND TO LAUNCH:**

You need to create a directory for the results and set the checkpoint and the batch size. 
Locally on laptop:

Go in TLIO/src folder, write "poetry shell", go back to TLIO directory and then launch:

python3 src/main_net.py --mode test --root_dir data/Dataset/ --test_list data/Dataset/test.txt --model_path results/Multiple_Traj/checkpoints/checkpoint_3.pt --out_dir results/Multiple_Traj/results/network/ --batch_size 1 --imu_freq 500 --save_plot --window_time 0.2


python3 src/main_net.py --mode test --root_dir data/Dataset_Multiple_Traj --test_list data/Dataset_Multiple_Traj/test.txt --model_path results/Multiple_Traj/checkpoints/checkpoint_8.pt --out_dir results/Multiple_Traj/results/network/ --batch_size 1 --imu_freq 500 --save_plot --window_time 0.5


python3 src/main_net.py --mode test --root_dir data/Dataset_Multiple_Traj_Old/ --test_list data/Dataset_Multiple_Traj_Old/test.txt --model_path results/Multiple_Traj/checkpoints/checkpoint_8.pt --out_dir results/Multiple_Traj/results/network/ --batch_size 1 --imu_freq 500 --save_plot --window_time 0.5

python3 src/main_net.py --mode test --root_dir data/Dataset_cpc_1_65/ --test_list data/Dataset_cpc_1_65/test.txt --model_path results/cpc_1_65_StepSize_wt0_6/checkpoints/checkpoint_24.pt --out_dir results/cpc_1_65_StepSize_wt0_6/results/network/ --batch_size 1 --imu_freq 500 --save_plot --window_time 0.6


python3 src/main_net.py --mode test --root_dir data/Dataset_Multiple_Traj_New --test_list data/Dataset_Multiple_Traj_New/test.txt --model_path results/Dataset_Multiple_Traj_New0_2/checkpoints/checkpoint_2.pt --out_dir results/Dataset_Multiple_Traj_New0_2/results/network/ --batch_size 1 --imu_freq 500 --save_plot --window_time 0.2


python3 src/main_net.py --mode test --root_dir data/CircleNoise500/ --test_list data/CircleNoise500/test.txt --model_path results/CircleNoise500/checkpoints/checkpoint_10.pt --out_dir results/CircleNoise500/results/network/ --batch_size 1 --imu_freq 500 --save_plot --window_time 0.5

python3 src/main_net.py --mode test --root_dir data/Lemniscate/ --test_list data/Lemniscate/test.txt --model_path results/Lemniscate/checkpoints/checkpoint_8.pt --out_dir results/Lemniscate/results/network/ --batch_size 1 --imu_freq 500 --save_plot --window_time 0.5

python3 src/main_net.py --mode test --root_dir data/Circle_No_Noise/ --test_list data/Circle_No_Noise/test.txt --model_path results/Circle_No_Noise_wt05_lr_e_05_MSE_moreValidation/checkpoints/checkpoint_755.pt --out_dir results/Circle_No_Noise_wt05_lr_e_05_MSE_moreValidation/results/network/ --batch_size 128 --imu_freq 500 --save_plot --window_time 0.5


python3 src/main_net.py --mode test --root_dir data/Dataset_Multiple_Traj50/ --test_list data/Dataset_Multiple_Traj50/test.txt --model_path results/MultipleCut50_wt05_lr_e_05_MSE/checkpoints/checkpoint_9.pt --out_dir results/MultipleCut50_wt05_lr_e_05_MSE/results/network/ --batch_size 128 --imu_freq 500 --save_plot --window_time 0.5


python3 src/main_net.py --mode test --root_dir data/OverfitCircle1000/ --test_list data/OverfitCircle1000/test.txt --model_path results/OverfitCircle1000/checkpoints/checkpoint_176.pt --out_dir results/OverfitCircle1000/results/network/ --batch_size 1 --imu_freq 500 --save_plot --window_time 0.5

python3 src/main_net.py --mode test --root_dir data/Dataset_Multiple_Traj50/ --test_list data/Dataset_Multiple_Traj50/test.txt --model_path results/Random_wt05_lr_e_05_MSE/checkpoints/checkpoint_259.pt --out_dir results/Random_wt05_lr_e_05_MSE/results/network/ --batch_size 1 --imu_freq 500 --save_plot --window_time 0.5


python3 src/main_net.py --mode test --root_dir data/Dataset_Multiple_Traj50/ --test_list data/Dataset_Multiple_Traj50/test.txt --model_path results/MultipleCut50_wt05_lr_e_05_MS/checkpoints/checkpoint_22.pt --out_dir results/MultipleCut50_wt05_lr_e_05_MSE/results/network/ --batch_size 1 --imu_freq 500 --save_plot --window_time 0.5

### Run EKF Filter and Network 

Go to TLIO and launch:

**COMMAND TO LAUNCH:**

python3 src/maiCircle_No_Noise_wt05_ML/n_filter.py --root_dir data/Dataset --data_list data/Dataset/test.txt --model_path results/race_track_18Jun21_StepSize/checkpoints/checkpoint_27.pt --model_param_path results/race_track_18Jun21_StepSize/parameters.json --out_dir results/race_track_18Jun21_StepSize/results/filter/ --update_freq 20 --initialize_with_offline_calib --erase_old_log


python3 src/main_filter.py --root_dir data/Dataset_cpc_1_65 --data_list data/Dataset_cpc_1_65/test.txt --model_path results/cpc_1_65_StepSize/checkpoints/checkpoint_84.pt --model_param_path results/cpc_1_65_StepSize/parameters.json --out_dir results/cpc_1_65_StepSize/results/filter/ --update_freq 20 --initialize_with_offline_calib --erase_old_log


python3 src/main_filter.py --root_dir data/Dataset --data_list data/Dataset/test.txt --model_path results/Multiple_Traj/checkpoints/checkpoint_3.pt --model_param_path results/Multiple_Traj/parameters.json --out_dir results/Multiple_Traj/results/filter/ --update_freq 20 --initialize_with_offline_calib --erase_old_log


python3 src/main_filter.py --root_dir data/Dataset_Multiple_Traj_Old --data_list data/Dataset_Multiple_Traj_Old/test.txt --model_path results/Multiple_Traj/checkpoints/checkpoint_8.pt --model_param_path results/Multiple_Traj/parameters.json --out_dir results/Multiple_Traj/results/filter/ --update_freq 20 --initialize_with_offline_calib --erase_old_log


python3 src/main_filter.py --root_dir data/Dataset_cpc_1_65 --data_list data/Dataset_cpc_1_65/test.txt --model_path results/cpc_1_65_StepSize_wt0_6/checkpoints/checkpoint_24.pt --model_param_path results/cpc_1_65_StepSize_wt0_6/parameters.json --out_dir results/cpc_1_65_StepSize_wt0_6/results/filter/ --update_freq 20 --initialize_with_offline_calib --erase_old_log


python3 src/main_filter.py --root_dir data/Dataset_Multiple_Traj_New --data_list data/Dataset_Multiple_Traj_New/test.txt --model_path results/Dataset_Multiple_Traj_New0_2/checkpoints/checkpoint_2.pt --model_param_path results/Dataset_Multiple_Traj_New0_2/parameters.json --out_dir results/Dataset_Multiple_Traj_New0_2/results/filter/ --update_freq 20 --initialize_with_offline_calib --erase_old_log

python3 src/main_filter.py --root_dir data/Lemniscate --data_list data/Lemniscate/test.txt --model_path results/Lemniscate/checkpoints/checkpoint_8.pt --model_param_path results/Lemniscate/parameters.json --out_dir results/Lemniscate/results/filter/ --update_freq 20 --initialize_with_offline_calib --erase_old_log

python3 src/main_filter.py --root_dir data/Dataset_Multiple_Traj50 --data_list data/Dataset_Multiple_Traj50/test.txt --model_path results/MultipleCut50_wt05_lr_e_05_MSE/checkpoints/checkpoint_9.pt --model_param_path results/MultipleCut50_wt05_lr_e_05_MSE/parameters.json --out_dir results/MultipleCut50_wt05_lr_e_05_MSE/results/filter/ --update_freq 20 --initialize_with_offline_calib --erase_old_log

python3 src/main_filter.py --root_dir data/Circle_No_Noise --data_list data/Circle_No_Noise/test.txt --model_path results/Circle_No_Noise_wt05_lr_e_05_MSE_moreValidation/checkpoints/checkpoint_755.pt --model_param_path results/Circle_No_Noise_wt05_lr_e_05_MSE_moreValidation/parameters.json --out_dir results/Circle_No_Noise_wt05_lr_e_05_MSE_moreValidation/results/filter/ --update_freq 20 --initialize_with_offline_calib --erase_old_log


python3 src/main_filter.py --root_dir data/OverfitCircle1000 --data_list data/OverfitCircle1000/test.txt --model_path results/OverfitCircle1000/checkpoints/checkpoint_199.pt --model_param_path results/OverfitCircle1000/parameters.json --out_dir results/OverfitCircle1000/results/filter/ --update_freq 20 --initialize_with_offline_calib --erase_old_log

python3 src/main_filter.py --root_dir data/Dataset_Multiple_Traj50 --data_list data/Dataset_Multiple_Traj50/test.txt --model_path results/Random_wt05_lr_e_05_MSE/checkpoints/checkpoint_259.pt --model_param_path results/Random_wt05_lr_e_05_MSE/parameters.json --out_dir results/Random_wt05_lr_e_05_MSE/results/filter/ --update_freq 20 --initialize_with_offline_calib --erase_old_log

python3 src/main_filter.py --root_dir data/Dataset_Multiple_Traj50 --data_list data/Dataset_Multiple_Traj50/test.txt --model_path results/MultipleCut50_wt05_lr_e_05_MSE/checkpoints/checkpoint_24.pt --model_param_path results/MultipleCut50_wt05_lr_e_05_MSE/parameters.json --out_dir results/MultipleCut50_wt05_lr_e_05_MSE/results/filter/ --update_freq 20 --initialize_with_offline_calib --erase_old_log

### Plot EKF Results

Go to TLIO/src and launch:

**COMMAND TO LAUNCH:**

python3 plot_filter_results.py --data_dir ../data/Dataset_cpc_1_65 --data_list ../data/Dataset_cpc_1_65/test.txt --filter_dir ../results/cpc_1_65_StepSize/results/filter/


python3 plot_filter_results.py --data_dir ../data/Dataset_Multiple_Traj_Old --data_list ../data/Dataset_Multiple_Traj_Old/test.txt --filter_dir ../results/Multiple_Traj/results/filter/

python3 plot_filter_results.py --data_dir ../data/Dataset_cpc_1_65 --data_list ../data/Dataset_cpc_1_65/test.txt --filter_dir ../results/cpc_1_65_StepSize_wt0_6/results/filter/

python3 plot_filter_results.py --data_dir ../data/Dataset_Multiple_Traj_New --data_list ../data/Dataset_Multiple_Traj_New/test.txt --filter_dir ../results/Dataset_Multiple_Traj_New0_2/results/filter/

python3 plot_filter_results.py --data_dir ../data/Lemniscate --data_list ../data/Lemniscate/test.txt --filter_dir ../results/Lemniscate/results/filter/

python3 plot_filter_results.py --data_dir ../data/Dataset_Multiple_Traj50 --data_list ../data/Dataset_Multiple_Traj50/test.txt --filter_dir ../results/MultipleCut50_wt05_lr_e_05_MSE/results/filter/

python3 plot_filter_results.py --data_dir ../data/Circle_No_Noise --data_list ../data/Circle_No_Noise/test.txt --filter_dir ../results/Circle_No_Noise_wt05_lr_e_05_MSE_moreValidation/results/filter/

python3 plot_filter_results.py --data_dir ../data/Dataset_Multiple_Traj50 --data_list ../data/Dataset_Multiple_Traj50/test.txt --filter_dir ../results/Random_wt05_lr_e_05_MSE/results/filter/

python3 plot_filter_results.py --data_dir ../data/Dataset_Multiple_Traj50 --data_list ../data/Dataset_Multiple_Traj50/test.txt --filter_dir ../results/MultipleCut50_wt05_lr_e_05_MSE/results/filter/



### SNAGA: Links to Wiki
- Machine learning hardware
https://app.gitbook.com/@rpg-uzh/s/rpg-uzh/computers-storage-and-printers/machine-learning-computer
- Work on SNAGA
https://app.gitbook.com/@rpg-uzh/s/rpg-uzh/computers-storage-and-printers/workshop-pcs-and-snaga
- Work on SNAGA from HOME
https://app.gitbook.com/@rpg-uzh/s/rpg-uzh/computers-storage-and-printers/workshop-pcs-and-snaga

### SNAGA: key concepts

**Connect to SNAGA**

- ssh aurora@snaga.ifi.uzh.ch

**Create a virtual environment on SNAGA**

- conda deactivate
- conda activate
- conda create --name TLIO
- conda activate TLIO

**Create a directory to store results and files**

In this case, it is:
- /data/storage/aurora/TLIO
Never store results in the HOME directory! 

**Create soft link between TLIO code in HOME and your results folder**

in ~TLIO: ln -s /data/storage/aurora/TLIO/data .
in ~TLIO: ln -s /data/storage/aurora/TLIO/results/ .

**Source ROS**

Add it to BASHRC in order to have it automatically
- source /opt/ros/melodic/setup.bash

**Rename on SNAGA**
- mv RaceTraj.bag raceTraj18Jun21.bag

**Create a COPY from local laptop to snaga**

Run the following on your local terminal:
- scp raceTraj18Jun21.bag aurora@snaga.ifi.uzh.ch:/data/storage/aurora/TLIO/data/rosbags 


**Create a COPY from snaga to local laptop**

- scp -r aurora@snaga.ifi.uzh.ch:/home/aurora/TLIO/results/race18June21 . 

**Check GPU usage and set a gpu for training**

- gpustat
- export CUDA_VISIBLE_DEVICES=X ---> where X is the number of a free gpu
- screen 
- press enter
"Screen" is needed to avoid killing the script if and when connection is lost.

**Exit SCREEN and SNAGA**

- exit
- exit






