# Point Clouds Registration with Probabilistic Data Association

This is an implementation of the Probabilistic Point Clouds Registration algorithm presented at IROS 2016, an ICP-like registration algorithm that uses a novel data association strategy derived from a probabilistic model. The result is a technique particularly robust against noise and outliers.

If you use this software for your work, please cite the following paper:
>G. Agamennoni, S. Fontana, R. Y. Siegwart and D. G. Sorrenti, "Point Clouds Registration with Probabilistic Data Association," 2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Daejeon, 2016, pp. 4092-4098.

## Instructions
### Compiling
Dependencies:
- A C++14 compiler
- Boost
- PCL
- OpenMP (not necessary, used only to parallelize the computation)
- [tclap](http://tclap.sourceforge.net/)
- Ceres
- Cmake
- GTest (optional, for tests)

This software uses [cmake](cmake.org) to ease the compilation:
~~~~ 
cd [code base folder]
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
~~~~

### Execution
~~~~
   ./probabilistic_point_cloud_registration  [--dump] [-g <string>] [-v]
                                        [-u] [-n <int>] [-c <float>] [-r
                                        <float>] [-d <float>] [-i <int>]
                                        [-m <int>] [-t <float>] [-s
                                        <float>] [--] [--version] [-h]
                                        <source_cloud> <target_cloud>


Where: 

   --dump
     Dump registration data to file

   -g <string>,  --ground_truth <string>
     The path of the ground truth for the source cloud, 
      that is, the source cloud in the ground truth pose, 
      if available. Used only to evaluate the result of the registration

   -v,  --verbose
     Verbosity

   -u,  --use_gaussian
     Whether to use a gaussian instead the a t-distribution for the probabilistic weights

   -n <int>,  --num_drop_iter <int>
     The maximum number of iterations during which the cost drop is allowed
     to be under cost_drop_thresh. 
      The algorithm terminate when either it reaches the num_iter iterations, 
       or the cost_drop has been below cost_drop_treshold for num_drop_iter

   -c <float>,  --cost_drop_treshold <float>
     If the cost_drop drops below this threshold for too many iterations,
     the algorithm terminate

   -r <float>,  --radius <float>
     The radius of the closest point neighborhood search

   -d <float>,  --dof <float>
     The Degree of freedom of t-distribution of the probabilistic weights

   -i <int>,  --num_iter <int>
     The maximum number of iterations to perform. T
      he algorithm terminate when either it reaches the num_iter iterations, 
      or the cost_drop has been below cost_drop_treshold for num_drop_iter

   -m <int>,  --max_neighbours <int>
     The max cardinality of the neighbours' set, that is, 
      the maximum number of neighbours for each point in the source point cloud

   -t <float>,  --target_filter_size <float>
     The leaf size of the voxel filter of the target cloud

   -s <float>,  --source_filter_size <float>
     The leaf size of the voxel filter of the source cloud
   -h,  --help
     Displays usage information and exits.

   <source_cloud>
     (required)  The path of the source point cloud, in pcd format

   <target_cloud>
     (required)  The path of the target point cloud, in pcd format
  ~~~~ 
