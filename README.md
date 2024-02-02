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