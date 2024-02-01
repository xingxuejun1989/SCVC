#PPF_Registration
Using PPF to achieve point cloud registration.

test environment：
pcl 1.12.1
vs 2019
cmake  3.15.2

Notice:
1.We need to manually st the boost path when we camke.

example:
E:/PCL 1.12.1/3rdParty/Boost/lib/cmake/Boost-1.78.0

2.Modify the floor function on line 80-81 of the PCL file  (pcl-1.12\pcl\registration\impl\ppf_registration.hpp)：

  const auto aux_size = static_cast<std::size_t>(std::ceil(2 * M_PI / search_method_->getAngleDiscretizationStep()));
