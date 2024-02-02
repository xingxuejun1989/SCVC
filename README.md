# Efficient Single Correspondence Voting for Point Cloud Registration

C++ implementation of the paper: [Efficient Single Correspondence Voting for Point Cloud Registration]

## Introduction
We are actively updating the code and will open source the paper's code after the article is officially released.

## Build
1.Before compiling, make sure CMAKE and VS are available.

2.The available process is to directly use CMAKE to generate the sln of vs, and then use vs to compile the code. The code relies on three open source libraries: Eigen, FLANN, and ICP_Princeton, which we have added to the source code.

3.Set the path parameters in the main function according to the path of the registration data.

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

### 3DMatch (1,279 pairs)

| No|	Featrues	|RTE(cm)	|RRE(°)	|RR	|MR	|Times(s)|
|:---|:---------:|:--------:|:--------:|:--------:|:--------:|:--------:|
|1 |FCGF |6.86 |1.78 |0.9523 |0.9437 |1.21|
|2 |FCGF |6.88 |1.79 |0.9539 |0.9450 |1.24|
|3 |FCGF |6.82 |1.79 |0.9539 |0.9450 |1.18|
|4 |FCGF |6.87 |1.78 |0.9531 |0.9444 |1.24|
|5 |FCGF |6.89 |1.80 |0.9547 |0.9455 |1.21|
|1 |FPFH |6.58 |1.72 |0.9203 |0.9133 |1.28|
|2 |FPFH |6.68 |1.75 |0.9265 |0.9185 |1.24|
|3 |FPFH |6.55 |1.73 |0.9195 |0.9161 |1.22|
|4 |FPFH |6.60 |1.73 |0.9218 |0.9175 |1.22|
|5 |FPFH |6.56 |1.72 |0.9195 |0.9129 |1.24|





### 3DLoMatch (1,726 pairs)

| No|	Featrues	|RTE(cm)	|RRE(°)	|RR	|MR	|Times(s)|
|:---|:---------:|:--------:|:--------:|:--------:|:--------:|:--------:|
|1 |FCGF |9.34 |2.63 |0.6454 |0.6208 |1.21|
|2 |FCGF |9.29 |2.61 |0.6501 |0.6290 |1.20|
|3 |FCGF |9.23 |2.59 |0.6437 |0.6161 |1.22|
|4 |FCGF |9.27 |2.59 |0.6466 |0.6243 |1.20|
|5 |FCGF |9.29 |2.60 |0.6530 |0.6346 |1.21|
|1 |FPFH |9.07 |2.62 |0.6246 |0.6139 |1.20|
|2 |FPFH |9.14 |2.63 |0.6263 |0.6171 |1.17|
|3 |FPFH |9.12 |2.63 |0.6228 |0.6119 |1.19|
|4 |FPFH |9.10 |2.64 |0.6240 |0.6135 |1.27|
|5 |FPFH |9.09 |2.61 |0.6222 |0.6145 |1.27|





### KITTI (555 pairs)
| No|	Featrues	|RTE(cm)	|RRE(°)	|RR	|MR	|Times(s)|
|:---|:---------:|:--------:|:--------:|:--------:|:--------:|:--------:|
|1 |FCGF |4.44 |0.14 |1.0000 |1.0000 |1.12|
|2 |FCGF |4.42 |0.14 |1.0000 |1.0000 |1.08|
|3 |FCGF |4.44 |0.14 |1.0000 |1.0000 |1.09|
|4 |FCGF |4.44 |0.14 |1.0000 |1.0000 |1.11|
|5 |FCGF |4.43 |0.15 |1.0000 |1.0000 |1.12|
|1 |FPFH |4.39 |0.15 |1.0000 |1.0000 |1.19|
|2 |FPFH |4.50 |0.15 |1.0000 |1.0000 |1.13|
|3 |FPFH |4.61 |0.15 |1.0000 |1.0000 |1.13|
|4 |FPFH |4.47 |0.15 |1.0000 |1.0000 |1.14|
|5 |FPFH |4.42 |0.15 |1.0000 |1.0000 |1.13|



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