Recorded datasets (`data_dagger.pkl.gzip`) are stored here when `--data_dir` points to this folder.

The default location is `~/driftmodel/data`. The dataset is appended to across runs of `dagger.py`.

A Noetic dataset cannot be used: its IMU acceleration columns were wrong and its actions have 3 values instead of 2.
