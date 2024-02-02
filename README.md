# Efficient Single Correspondence Voting for Point Cloud Registration

C++ implementation of the paper: [Efficient Single Correspondence Voting for Point Cloud Registration]

## Introduction
We are actively updating the code and will open source the paper's code after the article is officially released.

## Build
Before compiling, make sure CMAKE and VS are available.

The available process is to directly use CMAKE to generate the sln of vs, and then use vs to compile the code. The code relies on three open source libraries: Eigen, FLANN, and ICP_Princeton, which we have added to the source code.

Code has been tested with  Win11, VS2019, CMAKE 3.24, and OpenMP.

## Datasets download

The data required for testing can be obtained in the following ways:
### 3DMatch and 3DLoMatch

Point cloud data can be downloaded from the website: https://3dmatch.cs.princeton.edu/#rgbd-reconstruction-datasets.

The download address of the required features and benchmarks: https://drive.google.com/drive/folders/17oapnRqExtRyLMws98DgghZNWpPP7BBY?usp=sharing.

### KITTI
Point cloud data can be downloaded from the website: https://www.cvlibs.net/datasets/kitti/eval_odometry.php.

The download address of the required features and ICP fine optimization results: https://drive.google.com/drive/folders/1sT6xMCCiBEd-y3rPg5-qzNMEkANh55Rs?usp=sharing.



## Results

### 3DMatch

| No|	Featrues	|RTE(cm)	|RRE(°)	|RR	|MR	|Times(s)|
|:---|:---------:|:--------:|:--------:|:--------:|:--------:|:--------:|
|1 |FCGF |6.85745 |1.77614 |0.952307 |0.943658 |1.20518|
|2 |FCGF |6.87752 |1.78658 |0.95387 |0.944978 |1.24206|
|3 |FCGF |6.82178 |1.78587 |0.95387 |0.944978 |1.18448|
|4 |FCGF |6.87438 |1.78252 |0.953088 |0.944444 |1.24133|
|5 |FCGF |6.88747 |1.79995 |0.954652 |0.945513 |1.21201|
|6 |FCGF |6.8626 |1.78679 |0.955434 |0.946551 |1.20418|
|7 |FCGF |6.85038 |1.79015 |0.952307 |0.943013 |1.19815|
|8 |FCGF |6.86295 |1.7831 |0.955434 |0.946299 |1.2756|
|9 |FCGF |6.88174 |1.7949 |0.953088 |0.944192 |1.19476|
|10 |FCGF |6.8572 |1.78195 |0.95387 |0.945482 |1.18919|
|1 |FPFH |6.5764 |1.72187 |0.92025 |0.913272 |1.278|
|2 |FPFH |6.67973 |1.74767 |0.926505 |0.91845 |1.23604|
|3 |FPFH |6.54744 |1.73046 |0.919468 |0.916076 |1.21749|
|4 |FPFH |6.60071 |1.73054 |0.921814 |0.917538 |1.22337|
|5 |FPFH |6.5614 |1.71748 |0.919468 |0.912932 |1.23793|
|6 |FPFH |6.60713 |1.73121 |0.921032 |0.915405 |1.22113|
|7 |FPFH |6.60421 |1.73022 |0.921814 |0.91693 |1.239|
|8 |FPFH |6.54662 |1.72727 |0.918686 |0.911444 |1.24808|
|9 |FPFH |6.56145 |1.72761 |0.921032 |0.916102 |1.24822|
|10 |FPFH |6.5418 |1.72902 |0.919468 |0.91334 |1.25248|


### 3DLoMatch



### KITTI


## Citation

```bibtex
@InProceedings{XingEfficient2024,
    author    = { Xuejun Xing, Zhengda Lu, Yiqun Wang, Jun Xiao.},
    title     = {Efficient Single Correspondence Voting for Point Cloud Registration},
    booktitle = {IEEE Transactions on Image Processing},
    month     = {2},
    year      = {2024},
    pages     = {1-16}
}
```

## Acknowledgements

- [A Symmetric Objective Function for ICP](https://gfx.cs.princeton.edu/pubs/Rusinkiewicz_2019_ASO/)