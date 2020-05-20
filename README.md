# SFND-Term1-P5-Unscented-Kalman-Filter
Project 5 of Udacity Sensor Fusion Nanodegree

<img src="src/Term1-Project5-Unscented-Kalman-Filter.gif" width="700" height="400" />

## (TODO)Overview  
In this project you will implement an Unscented Kalman Filter to estimate the state of multiple cars on a highway using noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

The main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ukf_highway

Note that the programs that need to be written to accomplish the project are src/ukf.cpp, and src/ukf.h

The program main.cpp has already been filled out, but feel free to modify it.

<img src="media/ukf_highway.png" width="700" height="400" />

`main.cpp` is using `highway.h` to create a straight 3 lane highway environment with 3 traffic cars and the main ego car at the center. 
The viewer scene is centered around the ego car and the coordinate system is relative to the ego car as well. The ego car is green while the 
other traffic cars are blue. The traffic cars will be accelerating and altering their steering to change lanes. Each of the traffic car's has
it's own UKF object generated for it, and will update each indidual one during every time step. 

The red spheres above cars represent the (x,y) lidar detection and the purple lines show the radar measurements with the velocity magnitude along the detected angle. The Z axis is not taken into account for tracking, so you are only tracking along the X/Y axis.

---

## Prerequisites/Dependencies  
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
 * PCL 1.2

## Setup Instructions (abbreviated)  

Meet the [`Prerequisites/Dependencies`](/README.md#L32)  

### Ubuntu 

```bash
$> sudo apt install libpcl-dev
```

### Windows 

http://www.pointclouds.org/downloads/windows.html

### MAC

#### Install via Homebrew
1. install [homebrew](https://brew.sh/)
2. update homebrew 
	```bash
	$> brew update
	```
3. add  homebrew science [tap](https://docs.brew.sh/Taps) 
	```bash
	$> brew tap brewsci/science
	```
4. view pcl install options
	```bash
	$> brew options pcl
	```
5. install PCL 
	```bash
	$> brew install pcl
	```

#### Prebuilt Binaries via Universal Installer
http://www.pointclouds.org/downloads/macosx.html  
NOTE: very old version 

#### Build from Source

[PCL Source Github](https://github.com/PointCloudLibrary/pcl)

[PCL Mac Compilation Docs](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php)  

## Generating Additional Data  
This is optional!

If you'd like to generate your own radar and lidar modify the code in `highway.h` to alter the cars. Also check out `tools.cpp` to
change how measurements are taken, for instance lidar markers could be the (x,y) center of bounding boxes by scanning the PCD environment
and performing clustering. This is similar to what was done in Sensor Fusion Lidar Obstacle Detection.

## (TODO)Project Description  
.SFND-Term1-P5-Unscented-Kalman-Filter
├── CMakeLists.txt
├── media
│   ├── ukf_highway.png
│   └── ukf_highway_tracked.gif
├── README.md
├── src
│   ├── Eigen
│   │   ├── Array
│   │   ├── Cholesky
│   │   ├── CholmodSupport
│   │   ├── CMakeLists.txt
│   │   ├── Core
│   │   ├── Dense
│   │   ├── Eigen
│   │   ├── Eigen2Support
│   │   ├── Eigenvalues
│   │   ├── Geometry
│   │   ├── Householder
│   │   ├── IterativeLinearSolvers
│   │   ├── Jacobi
│   │   ├── LeastSquares
│   │   ├── LU
│   │   ├── MetisSupport
│   │   ├── OrderingMethods
│   │   ├── PardisoSupport
│   │   ├── PaStiXSupport
│   │   ├── QR
│   │   ├── QtAlignedMalloc
│   │   ├── Sparse
│   │   ├── SparseCholesky
│   │   ├── SparseCore
│   │   ├── SparseLU
│   │   ├── SparseQR
│   │   ├── SPQRSupport
│   │   ├── src
│   │   │   ├── Cholesky
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── LDLT.h
│   │   │   │   ├── LLT.h
│   │   │   │   └── LLT_MKL.h
│   │   │   ├── CholmodSupport
│   │   │   │   ├── CholmodSupport.h
│   │   │   │   └── CMakeLists.txt
│   │   │   ├── CMakeLists.txt
│   │   │   ├── Core
│   │   │   │   ├── arch
│   │   │   │   │   ├── AltiVec
│   │   │   │   │   │   ├── CMakeLists.txt
│   │   │   │   │   │   ├── Complex.h
│   │   │   │   │   │   └── PacketMath.h
│   │   │   │   │   ├── CMakeLists.txt
│   │   │   │   │   ├── Default
│   │   │   │   │   │   ├── CMakeLists.txt
│   │   │   │   │   │   └── Settings.h
│   │   │   │   │   ├── NEON
│   │   │   │   │   │   ├── CMakeLists.txt
│   │   │   │   │   │   ├── Complex.h
│   │   │   │   │   │   └── PacketMath.h
│   │   │   │   │   └── SSE
│   │   │   │   │       ├── CMakeLists.txt
│   │   │   │   │       ├── Complex.h
│   │   │   │   │       ├── MathFunctions.h
│   │   │   │   │       └── PacketMath.h
│   │   │   │   ├── ArrayBase.h
│   │   │   │   ├── Array.h
│   │   │   │   ├── ArrayWrapper.h
│   │   │   │   ├── Assign.h
│   │   │   │   ├── Assign_MKL.h
│   │   │   │   ├── BandMatrix.h
│   │   │   │   ├── Block.h
│   │   │   │   ├── BooleanRedux.h
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── CommaInitializer.h
│   │   │   │   ├── CoreIterators.h
│   │   │   │   ├── CwiseBinaryOp.h
│   │   │   │   ├── CwiseNullaryOp.h
│   │   │   │   ├── CwiseUnaryOp.h
│   │   │   │   ├── CwiseUnaryView.h
│   │   │   │   ├── DenseBase.h
│   │   │   │   ├── DenseCoeffsBase.h
│   │   │   │   ├── DenseStorage.h
│   │   │   │   ├── Diagonal.h
│   │   │   │   ├── DiagonalMatrix.h
│   │   │   │   ├── DiagonalProduct.h
│   │   │   │   ├── Dot.h
│   │   │   │   ├── EigenBase.h
│   │   │   │   ├── Flagged.h
│   │   │   │   ├── ForceAlignedAccess.h
│   │   │   │   ├── Functors.h
│   │   │   │   ├── Fuzzy.h
│   │   │   │   ├── GeneralProduct.h
│   │   │   │   ├── GenericPacketMath.h
│   │   │   │   ├── GlobalFunctions.h
│   │   │   │   ├── IO.h
│   │   │   │   ├── MapBase.h
│   │   │   │   ├── Map.h
│   │   │   │   ├── MathFunctions.h
│   │   │   │   ├── MatrixBase.h
│   │   │   │   ├── Matrix.h
│   │   │   │   ├── NestByValue.h
│   │   │   │   ├── NoAlias.h
│   │   │   │   ├── NumTraits.h
│   │   │   │   ├── PermutationMatrix.h
│   │   │   │   ├── PlainObjectBase.h
│   │   │   │   ├── ProductBase.h
│   │   │   │   ├── products
│   │   │   │   │   ├── CMakeLists.txt
│   │   │   │   │   ├── CoeffBasedProduct.h
│   │   │   │   │   ├── GeneralBlockPanelKernel.h
│   │   │   │   │   ├── GeneralMatrixMatrix.h
│   │   │   │   │   ├── GeneralMatrixMatrix_MKL.h
│   │   │   │   │   ├── GeneralMatrixMatrixTriangular.h
│   │   │   │   │   ├── GeneralMatrixMatrixTriangular_MKL.h
│   │   │   │   │   ├── GeneralMatrixVector.h
│   │   │   │   │   ├── GeneralMatrixVector_MKL.h
│   │   │   │   │   ├── Parallelizer.h
│   │   │   │   │   ├── SelfadjointMatrixMatrix.h
│   │   │   │   │   ├── SelfadjointMatrixMatrix_MKL.h
│   │   │   │   │   ├── SelfadjointMatrixVector.h
│   │   │   │   │   ├── SelfadjointMatrixVector_MKL.h
│   │   │   │   │   ├── SelfadjointProduct.h
│   │   │   │   │   ├── SelfadjointRank2Update.h
│   │   │   │   │   ├── TriangularMatrixMatrix.h
│   │   │   │   │   ├── TriangularMatrixMatrix_MKL.h
│   │   │   │   │   ├── TriangularMatrixVector.h
│   │   │   │   │   ├── TriangularMatrixVector_MKL.h
│   │   │   │   │   ├── TriangularSolverMatrix.h
│   │   │   │   │   ├── TriangularSolverMatrix_MKL.h
│   │   │   │   │   └── TriangularSolverVector.h
│   │   │   │   ├── Random.h
│   │   │   │   ├── Redux.h
│   │   │   │   ├── Ref.h
│   │   │   │   ├── Replicate.h
│   │   │   │   ├── ReturnByValue.h
│   │   │   │   ├── Reverse.h
│   │   │   │   ├── Select.h
│   │   │   │   ├── SelfAdjointView.h
│   │   │   │   ├── SelfCwiseBinaryOp.h
│   │   │   │   ├── SolveTriangular.h
│   │   │   │   ├── StableNorm.h
│   │   │   │   ├── Stride.h
│   │   │   │   ├── Swap.h
│   │   │   │   ├── Transpose.h
│   │   │   │   ├── Transpositions.h
│   │   │   │   ├── TriangularMatrix.h
│   │   │   │   ├── util
│   │   │   │   │   ├── BlasUtil.h
│   │   │   │   │   ├── CMakeLists.txt
│   │   │   │   │   ├── Constants.h
│   │   │   │   │   ├── DisableStupidWarnings.h
│   │   │   │   │   ├── ForwardDeclarations.h
│   │   │   │   │   ├── Macros.h
│   │   │   │   │   ├── Memory.h
│   │   │   │   │   ├── Meta.h
│   │   │   │   │   ├── MKL_support.h
│   │   │   │   │   ├── NonMPL2.h
│   │   │   │   │   ├── ReenableStupidWarnings.h
│   │   │   │   │   ├── StaticAssert.h
│   │   │   │   │   └── XprHelper.h
│   │   │   │   ├── VectorBlock.h
│   │   │   │   ├── VectorwiseOp.h
│   │   │   │   └── Visitor.h
│   │   │   ├── Eigen2Support
│   │   │   │   ├── Block.h
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── Cwise.h
│   │   │   │   ├── CwiseOperators.h
│   │   │   │   ├── Geometry
│   │   │   │   │   ├── AlignedBox.h
│   │   │   │   │   ├── All.h
│   │   │   │   │   ├── AngleAxis.h
│   │   │   │   │   ├── CMakeLists.txt
│   │   │   │   │   ├── Hyperplane.h
│   │   │   │   │   ├── ParametrizedLine.h
│   │   │   │   │   ├── Quaternion.h
│   │   │   │   │   ├── Rotation2D.h
│   │   │   │   │   ├── RotationBase.h
│   │   │   │   │   ├── Scaling.h
│   │   │   │   │   ├── Transform.h
│   │   │   │   │   └── Translation.h
│   │   │   │   ├── Lazy.h
│   │   │   │   ├── LeastSquares.h
│   │   │   │   ├── LU.h
│   │   │   │   ├── Macros.h
│   │   │   │   ├── MathFunctions.h
│   │   │   │   ├── Memory.h
│   │   │   │   ├── Meta.h
│   │   │   │   ├── Minor.h
│   │   │   │   ├── QR.h
│   │   │   │   ├── SVD.h
│   │   │   │   ├── TriangularSolver.h
│   │   │   │   └── VectorBlock.h
│   │   │   ├── Eigenvalues
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── ComplexEigenSolver.h
│   │   │   │   ├── ComplexSchur.h
│   │   │   │   ├── ComplexSchur_MKL.h
│   │   │   │   ├── EigenSolver.h
│   │   │   │   ├── GeneralizedEigenSolver.h
│   │   │   │   ├── GeneralizedSelfAdjointEigenSolver.h
│   │   │   │   ├── HessenbergDecomposition.h
│   │   │   │   ├── MatrixBaseEigenvalues.h
│   │   │   │   ├── RealQZ.h
│   │   │   │   ├── RealSchur.h
│   │   │   │   ├── RealSchur_MKL.h
│   │   │   │   ├── SelfAdjointEigenSolver.h
│   │   │   │   ├── SelfAdjointEigenSolver_MKL.h
│   │   │   │   └── Tridiagonalization.h
│   │   │   ├── Geometry
│   │   │   │   ├── AlignedBox.h
│   │   │   │   ├── AngleAxis.h
│   │   │   │   ├── arch
│   │   │   │   │   ├── CMakeLists.txt
│   │   │   │   │   └── Geometry_SSE.h
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── EulerAngles.h
│   │   │   │   ├── Homogeneous.h
│   │   │   │   ├── Hyperplane.h
│   │   │   │   ├── OrthoMethods.h
│   │   │   │   ├── ParametrizedLine.h
│   │   │   │   ├── Quaternion.h
│   │   │   │   ├── Rotation2D.h
│   │   │   │   ├── RotationBase.h
│   │   │   │   ├── Scaling.h
│   │   │   │   ├── Transform.h
│   │   │   │   ├── Translation.h
│   │   │   │   └── Umeyama.h
│   │   │   ├── Householder
│   │   │   │   ├── BlockHouseholder.h
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── Householder.h
│   │   │   │   └── HouseholderSequence.h
│   │   │   ├── IterativeLinearSolvers
│   │   │   │   ├── BasicPreconditioners.h
│   │   │   │   ├── BiCGSTAB.h
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── ConjugateGradient.h
│   │   │   │   ├── IncompleteLUT.h
│   │   │   │   └── IterativeSolverBase.h
│   │   │   ├── Jacobi
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   └── Jacobi.h
│   │   │   ├── LU
│   │   │   │   ├── arch
│   │   │   │   │   ├── CMakeLists.txt
│   │   │   │   │   └── Inverse_SSE.h
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── Determinant.h
│   │   │   │   ├── FullPivLU.h
│   │   │   │   ├── Inverse.h
│   │   │   │   ├── PartialPivLU.h
│   │   │   │   └── PartialPivLU_MKL.h
│   │   │   ├── MetisSupport
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   └── MetisSupport.h
│   │   │   ├── misc
│   │   │   │   ├── blas.h
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── Image.h
│   │   │   │   ├── Kernel.h
│   │   │   │   ├── Solve.h
│   │   │   │   └── SparseSolve.h
│   │   │   ├── OrderingMethods
│   │   │   │   ├── Amd.h
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── Eigen_Colamd.h
│   │   │   │   └── Ordering.h
│   │   │   ├── PardisoSupport
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   └── PardisoSupport.h
│   │   │   ├── PaStiXSupport
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   └── PaStiXSupport.h
│   │   │   ├── plugins
│   │   │   │   ├── ArrayCwiseBinaryOps.h
│   │   │   │   ├── ArrayCwiseUnaryOps.h
│   │   │   │   ├── BlockMethods.h
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── CommonCwiseBinaryOps.h
│   │   │   │   ├── CommonCwiseUnaryOps.h
│   │   │   │   ├── MatrixCwiseBinaryOps.h
│   │   │   │   └── MatrixCwiseUnaryOps.h
│   │   │   ├── QR
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── ColPivHouseholderQR.h
│   │   │   │   ├── ColPivHouseholderQR_MKL.h
│   │   │   │   ├── FullPivHouseholderQR.h
│   │   │   │   ├── HouseholderQR.h
│   │   │   │   └── HouseholderQR_MKL.h
│   │   │   ├── SparseCholesky
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── SimplicialCholesky.h
│   │   │   │   └── SimplicialCholesky_impl.h
│   │   │   ├── SparseCore
│   │   │   │   ├── AmbiVector.h
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── CompressedStorage.h
│   │   │   │   ├── ConservativeSparseSparseProduct.h
│   │   │   │   ├── MappedSparseMatrix.h
│   │   │   │   ├── SparseBlock.h
│   │   │   │   ├── SparseColEtree.h
│   │   │   │   ├── SparseCwiseBinaryOp.h
│   │   │   │   ├── SparseCwiseUnaryOp.h
│   │   │   │   ├── SparseDenseProduct.h
│   │   │   │   ├── SparseDiagonalProduct.h
│   │   │   │   ├── SparseDot.h
│   │   │   │   ├── SparseFuzzy.h
│   │   │   │   ├── SparseMatrixBase.h
│   │   │   │   ├── SparseMatrix.h
│   │   │   │   ├── SparsePermutation.h
│   │   │   │   ├── SparseProduct.h
│   │   │   │   ├── SparseRedux.h
│   │   │   │   ├── SparseSelfAdjointView.h
│   │   │   │   ├── SparseSparseProductWithPruning.h
│   │   │   │   ├── SparseTranspose.h
│   │   │   │   ├── SparseTriangularView.h
│   │   │   │   ├── SparseUtil.h
│   │   │   │   ├── SparseVector.h
│   │   │   │   ├── SparseView.h
│   │   │   │   └── TriangularSolver.h
│   │   │   ├── SparseLU
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── SparseLU_column_bmod.h
│   │   │   │   ├── SparseLU_column_dfs.h
│   │   │   │   ├── SparseLU_copy_to_ucol.h
│   │   │   │   ├── SparseLU_gemm_kernel.h
│   │   │   │   ├── SparseLU.h
│   │   │   │   ├── SparseLU_heap_relax_snode.h
│   │   │   │   ├── SparseLUImpl.h
│   │   │   │   ├── SparseLU_kernel_bmod.h
│   │   │   │   ├── SparseLU_Memory.h
│   │   │   │   ├── SparseLU_panel_bmod.h
│   │   │   │   ├── SparseLU_panel_dfs.h
│   │   │   │   ├── SparseLU_pivotL.h
│   │   │   │   ├── SparseLU_pruneL.h
│   │   │   │   ├── SparseLU_relax_snode.h
│   │   │   │   ├── SparseLU_Structs.h
│   │   │   │   ├── SparseLU_SupernodalMatrix.h
│   │   │   │   └── SparseLU_Utils.h
│   │   │   ├── SparseQR
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   └── SparseQR.h
│   │   │   ├── SPQRSupport
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   └── SuiteSparseQRSupport.h
│   │   │   ├── StlSupport
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── details.h
│   │   │   │   ├── StdDeque.h
│   │   │   │   ├── StdList.h
│   │   │   │   └── StdVector.h
│   │   │   ├── SuperLUSupport
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   └── SuperLUSupport.h
│   │   │   ├── SVD
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── JacobiSVD.h
│   │   │   │   ├── JacobiSVD_MKL.h
│   │   │   │   └── UpperBidiagonalization.h
│   │   │   └── UmfPackSupport
│   │   │       ├── CMakeLists.txt
│   │   │       └── UmfPackSupport.h
│   │   ├── StdDeque
│   │   ├── StdList
│   │   ├── StdVector
│   │   ├── SuperLUSupport
│   │   ├── SVD
│   │   └── UmfPackSupport
│   ├── highway.h
│   ├── main.cpp
│   ├── measurement_package.h
│   ├── render
│   │   ├── box.h
│   │   ├── render.cpp
│   │   └── render.h
│   ├── sensors
│   │   ├── data
│   │   │   └── pcd
│   │   │       ├── highway_0.pcd
│   │   │       ├── highway_1000000.pcd
│   │   │       ├── highway_100000.pcd
│   │   │       ├── highway_1033333.pcd
│   │   │       ├── highway_1066666.pcd
│   │   │       ├── highway_1100000.pcd
│   │   │       ├── highway_1133333.pcd
│   │   │       ├── highway_1166666.pcd
│   │   │       ├── highway_1200000.pcd
│   │   │       ├── highway_1233333.pcd
│   │   │       ├── highway_1266666.pcd
│   │   │       ├── highway_1300000.pcd
│   │   │       ├── highway_1333333.pcd
│   │   │       ├── highway_133333.pcd
│   │   │       ├── highway_1366666.pcd
│   │   │       ├── highway_1400000.pcd
│   │   │       ├── highway_1433333.pcd
│   │   │       ├── highway_1466666.pcd
│   │   │       ├── highway_1500000.pcd
│   │   │       ├── highway_1533333.pcd
│   │   │       ├── highway_1566666.pcd
│   │   │       ├── highway_1600000.pcd
│   │   │       ├── highway_1633333.pcd
│   │   │       ├── highway_1666666.pcd
│   │   │       ├── highway_166666.pcd
│   │   │       ├── highway_1700000.pcd
│   │   │       ├── highway_1733333.pcd
│   │   │       ├── highway_1766666.pcd
│   │   │       ├── highway_1800000.pcd
│   │   │       ├── highway_1833333.pcd
│   │   │       ├── highway_1866666.pcd
│   │   │       ├── highway_1900000.pcd
│   │   │       ├── highway_1933333.pcd
│   │   │       ├── highway_1966666.pcd
│   │   │       ├── highway_2000000.pcd
│   │   │       ├── highway_200000.pcd
│   │   │       ├── highway_2033333.pcd
│   │   │       ├── highway_2066666.pcd
│   │   │       ├── highway_2100000.pcd
│   │   │       ├── highway_2133333.pcd
│   │   │       ├── highway_2166666.pcd
│   │   │       ├── highway_2200000.pcd
│   │   │       ├── highway_2233333.pcd
│   │   │       ├── highway_2266666.pcd
│   │   │       ├── highway_2300000.pcd
│   │   │       ├── highway_2333333.pcd
│   │   │       ├── highway_233333.pcd
│   │   │       ├── highway_2366666.pcd
│   │   │       ├── highway_2400000.pcd
│   │   │       ├── highway_2433333.pcd
│   │   │       ├── highway_2466666.pcd
│   │   │       ├── highway_2500000.pcd
│   │   │       ├── highway_2533333.pcd
│   │   │       ├── highway_2566666.pcd
│   │   │       ├── highway_2600000.pcd
│   │   │       ├── highway_2633333.pcd
│   │   │       ├── highway_2666666.pcd
│   │   │       ├── highway_266666.pcd
│   │   │       ├── highway_2700000.pcd
│   │   │       ├── highway_2733333.pcd
│   │   │       ├── highway_2766666.pcd
│   │   │       ├── highway_2800000.pcd
│   │   │       ├── highway_2833333.pcd
│   │   │       ├── highway_2866666.pcd
│   │   │       ├── highway_2900000.pcd
│   │   │       ├── highway_2933333.pcd
│   │   │       ├── highway_2966666.pcd
│   │   │       ├── highway_3000000.pcd
│   │   │       ├── highway_300000.pcd
│   │   │       ├── highway_3033333.pcd
│   │   │       ├── highway_3066666.pcd
│   │   │       ├── highway_3100000.pcd
│   │   │       ├── highway_3133333.pcd
│   │   │       ├── highway_3166666.pcd
│   │   │       ├── highway_3200000.pcd
│   │   │       ├── highway_3233333.pcd
│   │   │       ├── highway_3266666.pcd
│   │   │       ├── highway_3300000.pcd
│   │   │       ├── highway_3333333.pcd
│   │   │       ├── highway_333333.pcd
│   │   │       ├── highway_33333.pcd
│   │   │       ├── highway_3366666.pcd
│   │   │       ├── highway_3400000.pcd
│   │   │       ├── highway_3433333.pcd
│   │   │       ├── highway_3466666.pcd
│   │   │       ├── highway_3500000.pcd
│   │   │       ├── highway_3533333.pcd
│   │   │       ├── highway_3566666.pcd
│   │   │       ├── highway_3600000.pcd
│   │   │       ├── highway_3633333.pcd
│   │   │       ├── highway_3666666.pcd
│   │   │       ├── highway_366666.pcd
│   │   │       ├── highway_3700000.pcd
│   │   │       ├── highway_3733333.pcd
│   │   │       ├── highway_3766666.pcd
│   │   │       ├── highway_3800000.pcd
│   │   │       ├── highway_3833333.pcd
│   │   │       ├── highway_3866666.pcd
│   │   │       ├── highway_3900000.pcd
│   │   │       ├── highway_3933333.pcd
│   │   │       ├── highway_3966666.pcd
│   │   │       ├── highway_4000000.pcd
│   │   │       ├── highway_400000.pcd
│   │   │       ├── highway_4033333.pcd
│   │   │       ├── highway_4066666.pcd
│   │   │       ├── highway_4100000.pcd
│   │   │       ├── highway_4133333.pcd
│   │   │       ├── highway_4166666.pcd
│   │   │       ├── highway_4200000.pcd
│   │   │       ├── highway_4233333.pcd
│   │   │       ├── highway_4266666.pcd
│   │   │       ├── highway_4300000.pcd
│   │   │       ├── highway_4333333.pcd
│   │   │       ├── highway_433333.pcd
│   │   │       ├── highway_4366666.pcd
│   │   │       ├── highway_4400000.pcd
│   │   │       ├── highway_4433333.pcd
│   │   │       ├── highway_4466666.pcd
│   │   │       ├── highway_4500000.pcd
│   │   │       ├── highway_4533333.pcd
│   │   │       ├── highway_4566666.pcd
│   │   │       ├── highway_4600000.pcd
│   │   │       ├── highway_4633333.pcd
│   │   │       ├── highway_4666666.pcd
│   │   │       ├── highway_466666.pcd
│   │   │       ├── highway_4700000.pcd
│   │   │       ├── highway_4733333.pcd
│   │   │       ├── highway_4766666.pcd
│   │   │       ├── highway_4800000.pcd
│   │   │       ├── highway_4833333.pcd
│   │   │       ├── highway_4866666.pcd
│   │   │       ├── highway_4900000.pcd
│   │   │       ├── highway_4933333.pcd
│   │   │       ├── highway_4966666.pcd
│   │   │       ├── highway_5000000.pcd
│   │   │       ├── highway_500000.pcd
│   │   │       ├── highway_5033333.pcd
│   │   │       ├── highway_5066666.pcd
│   │   │       ├── highway_5100000.pcd
│   │   │       ├── highway_5133333.pcd
│   │   │       ├── highway_5166666.pcd
│   │   │       ├── highway_5200000.pcd
│   │   │       ├── highway_5233333.pcd
│   │   │       ├── highway_5266666.pcd
│   │   │       ├── highway_5300000.pcd
│   │   │       ├── highway_5333333.pcd
│   │   │       ├── highway_533333.pcd
│   │   │       ├── highway_5366666.pcd
│   │   │       ├── highway_5400000.pcd
│   │   │       ├── highway_5433333.pcd
│   │   │       ├── highway_5466666.pcd
│   │   │       ├── highway_5500000.pcd
│   │   │       ├── highway_5533333.pcd
│   │   │       ├── highway_5566666.pcd
│   │   │       ├── highway_5600000.pcd
│   │   │       ├── highway_5633333.pcd
│   │   │       ├── highway_5666666.pcd
│   │   │       ├── highway_566666.pcd
│   │   │       ├── highway_5700000.pcd
│   │   │       ├── highway_5733333.pcd
│   │   │       ├── highway_5766666.pcd
│   │   │       ├── highway_5800000.pcd
│   │   │       ├── highway_5833333.pcd
│   │   │       ├── highway_5866666.pcd
│   │   │       ├── highway_5900000.pcd
│   │   │       ├── highway_5933333.pcd
│   │   │       ├── highway_5966666.pcd
│   │   │       ├── highway_6000000.pcd
│   │   │       ├── highway_600000.pcd
│   │   │       ├── highway_6033333.pcd
│   │   │       ├── highway_6066666.pcd
│   │   │       ├── highway_6100000.pcd
│   │   │       ├── highway_6133333.pcd
│   │   │       ├── highway_6166666.pcd
│   │   │       ├── highway_6200000.pcd
│   │   │       ├── highway_6233333.pcd
│   │   │       ├── highway_6266666.pcd
│   │   │       ├── highway_6300000.pcd
│   │   │       ├── highway_6333333.pcd
│   │   │       ├── highway_633333.pcd
│   │   │       ├── highway_6366666.pcd
│   │   │       ├── highway_6400000.pcd
│   │   │       ├── highway_6433333.pcd
│   │   │       ├── highway_6466666.pcd
│   │   │       ├── highway_6500000.pcd
│   │   │       ├── highway_6533333.pcd
│   │   │       ├── highway_6566666.pcd
│   │   │       ├── highway_6600000.pcd
│   │   │       ├── highway_6633333.pcd
│   │   │       ├── highway_6666666.pcd
│   │   │       ├── highway_666666.pcd
│   │   │       ├── highway_66666.pcd
│   │   │       ├── highway_6700000.pcd
│   │   │       ├── highway_6733333.pcd
│   │   │       ├── highway_6766666.pcd
│   │   │       ├── highway_6800000.pcd
│   │   │       ├── highway_6833333.pcd
│   │   │       ├── highway_6866666.pcd
│   │   │       ├── highway_6900000.pcd
│   │   │       ├── highway_6933333.pcd
│   │   │       ├── highway_6966666.pcd
│   │   │       ├── highway_7000000.pcd
│   │   │       ├── highway_700000.pcd
│   │   │       ├── highway_7033333.pcd
│   │   │       ├── highway_7066666.pcd
│   │   │       ├── highway_7100000.pcd
│   │   │       ├── highway_7133333.pcd
│   │   │       ├── highway_7166666.pcd
│   │   │       ├── highway_7200000.pcd
│   │   │       ├── highway_7233333.pcd
│   │   │       ├── highway_7266666.pcd
│   │   │       ├── highway_7300000.pcd
│   │   │       ├── highway_7333333.pcd
│   │   │       ├── highway_733333.pcd
│   │   │       ├── highway_7366666.pcd
│   │   │       ├── highway_7400000.pcd
│   │   │       ├── highway_7433333.pcd
│   │   │       ├── highway_7466666.pcd
│   │   │       ├── highway_7500000.pcd
│   │   │       ├── highway_7533333.pcd
│   │   │       ├── highway_7566666.pcd
│   │   │       ├── highway_7600000.pcd
│   │   │       ├── highway_7633333.pcd
│   │   │       ├── highway_7666666.pcd
│   │   │       ├── highway_766666.pcd
│   │   │       ├── highway_7700000.pcd
│   │   │       ├── highway_7733333.pcd
│   │   │       ├── highway_7766666.pcd
│   │   │       ├── highway_7800000.pcd
│   │   │       ├── highway_7833333.pcd
│   │   │       ├── highway_7866666.pcd
│   │   │       ├── highway_7900000.pcd
│   │   │       ├── highway_7933333.pcd
│   │   │       ├── highway_7966666.pcd
│   │   │       ├── highway_8000000.pcd
│   │   │       ├── highway_800000.pcd
│   │   │       ├── highway_8033333.pcd
│   │   │       ├── highway_8066666.pcd
│   │   │       ├── highway_8100000.pcd
│   │   │       ├── highway_8133333.pcd
│   │   │       ├── highway_8166666.pcd
│   │   │       ├── highway_8200000.pcd
│   │   │       ├── highway_8233333.pcd
│   │   │       ├── highway_8266666.pcd
│   │   │       ├── highway_8300000.pcd
│   │   │       ├── highway_8333333.pcd
│   │   │       ├── highway_833333.pcd
│   │   │       ├── highway_8366666.pcd
│   │   │       ├── highway_8400000.pcd
│   │   │       ├── highway_8433333.pcd
│   │   │       ├── highway_8466666.pcd
│   │   │       ├── highway_8500000.pcd
│   │   │       ├── highway_8533333.pcd
│   │   │       ├── highway_8566666.pcd
│   │   │       ├── highway_8600000.pcd
│   │   │       ├── highway_8633333.pcd
│   │   │       ├── highway_8666666.pcd
│   │   │       ├── highway_866666.pcd
│   │   │       ├── highway_8700000.pcd
│   │   │       ├── highway_8733333.pcd
│   │   │       ├── highway_8766666.pcd
│   │   │       ├── highway_8800000.pcd
│   │   │       ├── highway_8833333.pcd
│   │   │       ├── highway_8866666.pcd
│   │   │       ├── highway_8900000.pcd
│   │   │       ├── highway_8933333.pcd
│   │   │       ├── highway_8966666.pcd
│   │   │       ├── highway_9000000.pcd
│   │   │       ├── highway_900000.pcd
│   │   │       ├── highway_9033333.pcd
│   │   │       ├── highway_9066666.pcd
│   │   │       ├── highway_9100000.pcd
│   │   │       ├── highway_9133333.pcd
│   │   │       ├── highway_9166666.pcd
│   │   │       ├── highway_9200000.pcd
│   │   │       ├── highway_9233333.pcd
│   │   │       ├── highway_9266666.pcd
│   │   │       ├── highway_9300000.pcd
│   │   │       ├── highway_9333333.pcd
│   │   │       ├── highway_933333.pcd
│   │   │       ├── highway_9366666.pcd
│   │   │       ├── highway_9400000.pcd
│   │   │       ├── highway_9433333.pcd
│   │   │       ├── highway_9466666.pcd
│   │   │       ├── highway_9500000.pcd
│   │   │       ├── highway_9533333.pcd
│   │   │       ├── highway_9566666.pcd
│   │   │       ├── highway_9600000.pcd
│   │   │       ├── highway_9633333.pcd
│   │   │       ├── highway_9666666.pcd
│   │   │       ├── highway_966666.pcd
│   │   │       ├── highway_9700000.pcd
│   │   │       ├── highway_9733333.pcd
│   │   │       ├── highway_9766666.pcd
│   │   │       ├── highway_9800000.pcd
│   │   │       ├── highway_9833333.pcd
│   │   │       ├── highway_9866666.pcd
│   │   │       ├── highway_9900000.pcd
│   │   │       ├── highway_9933333.pcd
│   │   │       └── highway_9966666.pcd
│   │   └── lidar.h
│   ├── tools.cpp
│   ├── tools.h
│   ├── ukf.cpp
│   └── ukf.h
└── videos
    ├── Term1-Project5-Unscented-Kalman-Filter.gif
    └── Term1-Project5-Unscented-Kalman-Filter.mp4

## Run the project  
* Clone this repository  
```
git clone https://github.com/jinchaolu/SFND-Term1-P5-Unscented-Kalman-Filter.git
```
* Navigate to the `SFND-Term1-P5-Unscented-Kalman-Filter` folder  
```
cd SFND-Term1-P1-Lidar-Obstacle-Detection
```
* Create and open `build` folder  
```
mkdir build && cd build
```
* Compile your code  
```
cmake .. && make
```
* Run `ukf_highway` application  
```
./ukf_highway
```

## Tips  
1. It's recommended to update and upgrade your environment before running the code.  
```
sudo apt-get update && sudo apt-get upgrade -y
```

## Editor Settings  
We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style  
Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).  

## Project Rubric  
### 1. Compiling and Testing  
#### 1.1 The submission must compile.  
Yes, it does compile.  

### 2. Code Efficiency  
#### 2.1 The methods in the code should avoid unnecessary calculations.  
Yes, it does.  

### 3. Accuracy
#### 3.1 px, py, vx, vy output coordinates must have an RMSE <= [0.30, 0.16, 0.95, 0.70] after running for longer than 1 second.  
Yes, it does.  

### 4. Follows the Correct Algorithm
#### 4.1 Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.  
Yes, it does.  
