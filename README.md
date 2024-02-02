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
|1 |FPFH |6.5764 |1.72187 |0.92025 |0.913272 |1.278|
|2 |FPFH |6.67973 |1.74767 |0.926505 |0.91845 |1.23604|
|3 |FPFH |6.54744 |1.73046 |0.919468 |0.916076 |1.21749|
|4 |FPFH |6.60071 |1.73054 |0.921814 |0.917538 |1.22337|
|5 |FPFH |6.5614 |1.71748 |0.919468 |0.912932 |1.23793|



### 3DLoMatch

| No|	Featrues	|RTE(cm)	|RRE(°)	|RR	|MR	|Times(s)|
|:---|:---------:|:--------:|:--------:|:--------:|:--------:|:--------:|
|1 |FCGF |9.33911 |2.63312 |0.645423 |0.620755 |1.2084|
|2 |FCGF |9.29353 |2.6069 |0.650058 |0.629002 |1.20271|
|3 |FCGF |9.23205 |2.59091 |0.643685 |0.616058 |1.21544|
|4 |FCGF |9.26743 |2.5855 |0.646582 |0.62429 |1.20484|
|5 |FCGF |9.29441 |2.60007 |0.652955 |0.634577 |1.21199|
|1 |FPFH |9.07031 |2.61699 |0.624565 |0.613892 |1.20017|
|2 |FPFH |9.14462 |2.62557 |0.626304 |0.617107 |1.16673|
|3 |FPFH |9.1169 |2.63125 |0.622827 |0.611882 |1.18961|
|4 |FPFH |9.09929 |2.6352 |0.623986 |0.613465 |1.27089|
|5 |FPFH |9.09405 |2.61015 |0.622248 |0.614472 |1.26675|



### KITTI
| No|	Featrues	|RTE(cm)	|RRE(°)	|RR	|MR	|Times(s)|
|:---|:---------:|:--------:|:--------:|:--------:|:--------:|:--------:|
|1 |FCGF |4.44182 |0.144823 |1 |1 |1.1175|
|2 |FCGF |4.42362 |0.143906 |1 |1 |1.0825|
|3 |FCGF |4.44096 |0.142978 |1 |1 |1.09451|
|4 |FCGF |4.43887 |0.14452 |1 |1 |1.11181|
|5 |FCGF |4.42775 |0.145002 |1 |1 |1.12249|
|1 |FPFH |4.39271 |0.148867 |1 |1 |1.18986|
|2 |FPFH |4.5022 |0.148845 |1 |1 |1.12717|
|3 |FPFH |4.60507 |0.149385 |1 |1 |1.13093|
|4 |FPFH |4.4674 |0.149672 |1 |1 |1.13712|
|5 |FPFH |4.42344 |0.148981 |1 |1 |1.13028|

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