import numpy as np
import matplotlib.pyplot as plt
import os
import pandas as pd
import subprocess
from pathlib import Path


def cross_correlation(
        s1,
        s2,
        increment=1,
        min_samples=None,
        ):
    '''
    Returns cross correlation coefficients (r), root mean squared errors (
    rmse), root median squared errors (rmedianse) across two signals s1 and s2.

    Applies sampling-based shifts by increment-size steps, retaining at least
    a minimum amount of samples (default: half the samples of the shortest
    sequence)

    Important: Assumes that both sequences have a regular and identical
    sampling (e.g. 400 Hz)

    Offset is applied to signal s2
    '''

    # Number of samples
    n1 = s1.shape[0]
    n2 = s2.shape[0]
    # Set minimum number of samples.
    if min_samples is None:
        min_samples = int(np.min([n1/2, n2/2]))
    # Indices
    i1 = np.arange(0, n1, 1)
    i2 = np.arange(0, n2, 1)
    # Sample offset candidates
    offset_candidates = np.arange(
        -(n2-min_samples),
        n1-min_samples,
        increment)
    corr_dict = {}
    corr_dict.setdefault('offset_samples', [])
    corr_dict.setdefault('r', [])
    corr_dict.setdefault('rmse', [])
    corr_dict.setdefault('rmedianse', [])
    corr_dict.setdefault('samples', [])
    for offset in offset_candidates:
        curr_i1 = i1
        curr_i2 = i2 + offset
        curr_imin = np.max([curr_i1[0], curr_i2[0]])
        curr_imax = np.min([curr_i1[-1], curr_i2[-1]])
        ind1 = (curr_i1 >= curr_imin) & (curr_i1 <= curr_imax)
        ind2 = (curr_i2 >= curr_imin) & (curr_i2 <= curr_imax)
        curr_s1 = s1[ind1]
        curr_s2 = s2[ind2]
        # print('Offset: {} samples'.format(offset))
        # print('NaN samples count: vicon {}, betaflight {}'.format(np.sum(
        #     np.isnan(
        #     curr_s1)), np.sum(np.isnan(curr_s2))))
        r = np.corrcoef(curr_s1, curr_s2)[0, 1]
        rmse = np.sqrt(np.nanmean((curr_s1 - curr_s2)**2))
        rmedianse = np.sqrt(np.nanmedian((curr_s1 - curr_s2)**2))
        corr_dict['offset_samples'].append(offset)
        corr_dict['r'].append(r)
        corr_dict['rmse'].append(rmse)
        corr_dict['rmedianse'].append(rmedianse)
        corr_dict['samples'].append(curr_s1.shape[0])
    c = pd.DataFrame(corr_dict)
    return c


def downsample(
        df,
        sr,
        ) -> pd.DataFrame:
    """Downsample a pandas dataframe using the time variable (t) and norm
    acceleration (an)."""
    x = df['t'].values
    xn = np.arange(x[0], x[-1]+1/sr, 1/sr)
    ddict = {}
    for n in ['t', 'an']:
        ddict[n] = np.interp(xn, x, df[n].values)
    return pd.DataFrame(ddict)


def position2velocity(
        t:'np.array (n,), time in sec',
        p:'np.array (n,3), position in m',
        w:'smoothing window in sec'=0.2,
        ) -> 'np.array (n,3), velocity in m/s':
    # Copy of the data
    curr_t = t.copy()
    curr_px = p.copy()[:,0]
    curr_py = p.copy()[:,1]
    curr_pz = p.copy()[:,2]
    # Sampling rate
    sr = 1 / np.nanmedian(np.diff(curr_t))
    # Make sampling regular
    x = curr_t.copy()
    xn = np.arange(curr_t[0], curr_t[-1]+1/sr, 1/sr)
    curr_t = np.interp(xn, x, curr_t)
    curr_px = np.interp(xn, x, curr_px)
    curr_py = np.interp(xn, x, curr_py)
    curr_pz = np.interp(xn, x, curr_pz)
    # Compute velocity
    curr_vx = np.append(np.diff(curr_px, n=1), [0.0]) * sr
    curr_vy = np.append(np.diff(curr_py, n=1), [0.0]) * sr
    curr_vz = np.append(np.diff(curr_pz, n=1), [0.0]) * sr
    # Smooth velocity
    window = "blackman"
    window_len = int(sr * w)
    curr_vx_smooth = smoothSignal(curr_vx, window_len=window_len,
        window=window)[window_len // 2:curr_px.shape[0] + window_len // 2]
    curr_vy_smooth = smoothSignal(curr_vy, window_len=window_len,
        window=window)[window_len // 2:curr_py.shape[0] + window_len // 2]
    curr_vz_smooth = smoothSignal(curr_vz, window_len=window_len,
        window=window)[window_len // 2:curr_pz.shape[0] + window_len // 2]
    # Restore original sampling
    vx_smooth = np.interp(t, curr_t, curr_vx_smooth)
    vy_smooth = np.interp(t, curr_t, curr_vy_smooth)
    vz_smooth = np.interp(t, curr_t, curr_vz_smooth)
    # Combine the output into one array
    v_smooth = np.hstack((vx_smooth.reshape((-1, 1)),
        np.hstack((vy_smooth.reshape((-1, 1)), vz_smooth.reshape((-1, 1))))))
    return v_smooth


def position2acceleration(
        t:'np.array (n,), time in sec',
        p:'np.array (n,3), position in m',
        w:'smoothing window in sec'=0.2,
        ) -> 'np.array (n,3), acceleration in m/s/s':
    # Copy of the data
    curr_t = t.copy()
    curr_px = p.copy()[:,0]
    curr_py = p.copy()[:,1]
    curr_pz = p.copy()[:,2]
    # Sampling rate
    sr = 1 / np.nanmedian(np.diff(curr_t))
    # Make sampling regular
    x = curr_t.copy()
    xn = np.arange(curr_t[0], curr_t[-1]+1/sr, 1/sr)
    curr_t = np.interp(xn, x, curr_t)
    curr_px = np.interp(xn, x, curr_px)
    curr_py = np.interp(xn, x, curr_py)
    curr_pz = np.interp(xn, x, curr_pz)
    # Compute acceleration
    curr_ax = np.append(np.diff(curr_px, n=2), [0.0, 0.0]) * (sr ** 2)
    curr_ay = np.append(np.diff(curr_py, n=2), [0.0, 0.0]) * (sr ** 2)
    curr_az = np.append(np.diff(curr_pz, n=2), [0.0, 0.0]) * (sr ** 2)
    # Smooth acceleration
    window = "blackman"
    window_len = int(sr * w)
    curr_ax_smooth = smoothSignal(curr_ax, window_len=window_len,
        window=window)[window_len // 2:curr_px.shape[0] + window_len // 2]
    curr_ay_smooth = smoothSignal(curr_ay, window_len=window_len,
        window=window)[window_len // 2:curr_py.shape[0] + window_len // 2]
    curr_az_smooth = smoothSignal(curr_az, window_len=window_len,
        window=window)[window_len // 2:curr_pz.shape[0] + window_len // 2]
    # Add gravity
    curr_az_smooth += 9.81
    # Restore original sampling
    ax_smooth = np.interp(t, curr_t, curr_ax_smooth)
    ay_smooth = np.interp(t, curr_t, curr_ay_smooth)
    az_smooth = np.interp(t, curr_t, curr_az_smooth)
    # Combine the output into one array
    a_smooth = np.hstack((ax_smooth.reshape((-1, 1)),
        np.hstack((ay_smooth.reshape((-1, 1)), az_smooth.reshape((-1, 1))))))
    return a_smooth


def rosbag2csv(
        inpath,
        outpath=None,
        ) -> None:
    '''
    Converts a vicon rosbag to separate csv files per tracked object and
    saves it to outpath.
    '''

    # Convert filepath to pathlib object.
    if isinstance(inpath, str):
        inpath = Path(inpath)
    # Set outpath
    if outpath is None:
        outpath = inpath.parent/(inpath.stem+'.csv')
    # Covert outpath to pathlib object.
    if isinstance(outpath,str):
        outpath=Path(outpath)
    # Get vicon topics
    out = subprocess.Popen(
        ['rosbag', 'info', inpath.as_posix()],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT)
    stdout, stderr = out.communicate()
    topics = []
    for s in str(stdout).split(' '):
        if s.find('/vicon/') > -1:
            topics.append(s)
    # Save vicon topic to csv
    for topic in topics:

        curr_outpath = (outpath.parent /
                   (outpath.stem +
                    '_' +
                    topic.replace('/vicon/', '') +
                    '.csv'))
        print(curr_outpath)
        if not curr_outpath.exists():
            print('..loading {}'.format(inpath))
            # Make output folder.
            if not curr_outpath.parent.exists():
                curr_outpath.parent.mkdir(parents=True, exist_ok=True)
            # Rosbag to csv
            cmd = ' '.join(
                ['rostopic', 'echo', topic, '-b', inpath.as_posix(),
                 '-p', '>', curr_outpath.as_posix()])
            os.system(cmd)
            # Reformat header and timestamps
            df = pd.read_csv(curr_outpath)
            # Convert time to seconds
            df['%time'] = df['%time'].values / 1000000000
            # Add time since start
            df['t'] = df['%time'].values - df['%time'].iloc[0]
            # Rename columns
            name_dict = {
                '%time': 'ts',
                't': 't',
                'field.header.seq': 'f',
                'field.pose.position.x': 'px',
                'field.pose.position.y': 'py',
                'field.pose.position.z': 'pz',
                'field.pose.orientation.x': 'qx',
                'field.pose.orientation.y': 'qy',
                'field.pose.orientation.z': 'qz',
                'field.pose.orientation.w': 'qw',
            }
            df = df.rename(columns=name_dict)
            # Select columns of interest
            df = df.loc[:, name_dict.values()]
            # Add Velocity
            v = position2velocity(t=df['t'].values,
                                  p=df.loc[:,('px','py','pz')].values)

            np.savetxt(os.getcwd() + '/../data/ViconVelocity.txt', v)
            np.savetxt(os.getcwd() + '/../data/ts.txt', df['t'].values)
            np.savetxt(os.getcwd() + '/../data/p.txt', df.loc[:,('px','py','pz')].values)

            df['vx']=v[:, 0]
            df['vy']=v[:, 1]
            df['vz']=v[:, 2]
            df['vn']=np.linalg.norm(v, axis=1)
            # Add Acceleration
            a = position2acceleration(t=df['t'].values,
                                      p=df.loc[:, ('px', 'py', 'pz')].values)
            df['ax'] = a[:, 0]
            df['ay'] = a[:, 1]
            df['az'] = a[:, 2]
            df['an'] = np.linalg.norm(a, axis=1)
            print('..saving {}'.format(curr_outpath))
            df.to_csv(curr_outpath, index=False)


def smoothSignal(
        x,
        window_len=11,
        window='hanning'
        ):
    """smooth the data using a window with requested size.
    This method is based on the convolution of a scaled window with the signal.
    The signal is prepared by introducing reflected copies of the signal
    (with the window size) in both ends so that transient parts are minimized
    in the begining and end part of the output signal.
    input:
        x: the input signal
        window_len: the dimension of the smoothing window; should be an odd integer
        window: the type of window from 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'
            flat window will produce a moving average smoothing.
    output:
        the smoothed signal
    example:
    t=linspace(-2,2,0.1)
    x=sin(t)+randn(len(t))*0.1
    y=smooth(x)
    see also:
    numpy.hanning, numpy.hamming, numpy.bartlett, numpy.blackman, numpy.convolve
    scipy.signal.lfilter
    TODO: the window parameter could be the window itself if an array instead of a string
    NOTE: length(output) != length(input), to correct this: return y[(window_len/2-1):-(window_len/2)] instead of just y.
    """

    if x.ndim != 1:
        raise ValueError("smooth only accepts 1 dimension arrays.")

    if x.size < window_len:
        raise ValueError("Input vector needs to be bigger than window size.")

    if window_len < 3:
        return x

    if not window in ['flat', 'hanning', 'hamming', 'bartlett', 'blackman']:
        raise ValueError("Window is on of 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'")

    s = np.r_[x[window_len - 1:0:-1], x, x[-1:-window_len:-1]]
    # print(len(s))
    if window == 'flat':  # moving average
        w = np.ones(window_len, 'd')
    else:
        w = eval('np.' + window + '(window_len)')

    y = np.convolve(w / w.sum(), s, mode='valid')
    return y

# Path to data.
PATH= os.getcwd() + '/../../'
bag_name = '13_43_38'

# Sampling rate.
SR=100

# Vicon rosbag to csv.
# Compute norm acceleration(+gravity) "an" from position data.
rosbag2csv(PATH+'Vicon/data/' + bag_name + '.bag')

# Load vicon data.
r=pd.read_csv(PATH+'Vicon/data/' + bag_name + '_parrot.csv')

# Downsample vicon data.
r=downsample(r,SR)

# Load betaflight data.
# Includes norm acceleration "an" from IMU. #change ts and an compute this one
b=pd.read_csv(PATH + 'Vicon/data/LOG00001.csv')

# Downsample betaflight data.
b=downsample(b,SR)

# Compute cross-correlation between vicon and betaflight norm accelerations.
c=cross_correlation(s1=r['an'].values,s2=b['an'].values)

# Find peak correlation.
i=np.nanargmax(c['r'].values)
peak_correlation = c['r'].iloc[i]
offset_samples = c['offset_samples'].iloc[i]
offset_time = offset_samples / SR
string=('Cross-Correlation peak: r={:.2f}, offset={:.2f}sec [{:.0f} '
        'samples]'.format(peak_correlation,offset_time,offset_samples))
print(string)

# Apply offset to betaflight data.
bsync = b.copy()
bsync['t']+=offset_time

# Plot the results
plt.figure()
plt.gcf().set_figwidth(8)
plt.gcf().set_figheight(6)
plt.subplot(2,1,1)
plt.plot(b['t'],b['an'],label='betaflight')
plt.plot(r['t'],r['an'],label='vicon')
plt.ylim((0,60))
plt.xlabel('Time [s]')
plt.ylabel('Norm. Acceleration [m/s/s]')
plt.legend()
plt.title('Before Sync')
plt.subplot(2,1,2)
plt.plot(bsync['t'],bsync['an'],label='betaflight')
plt.plot(r['t'],r['an'],label='vicon')
plt.ylim((0,60))
plt.xlabel('Time [s]')
plt.ylabel('Norm. Acceleration [m/s/s]')
plt.legend()
plt.title('After Sync\n{}'.format(string))
plt.tight_layout()
plt.savefig(PATH+'Vicon/data/result.png')
plt.show()
