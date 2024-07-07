# bundle_adjustment_on_manifold

### Running with Docker
```shell
# First things first xD
cd bundle_adjustment_on_manifold

# Build the docker image from Dockerfile
docker build -t bundle_adjuster .

# Create a container from the image with a name. Note that you need to run this only once.
docker create --privileged -it -v ${PWD}:/home/bundle_adjustment_on_manifold -v /dev:/dev --name my_bundle_adjuster bundle_adjuster

# Start the container
docker start my_bundle_adjuster

# Enter into the container
docker exec -it my_bundle_adjuster bash
```

### Run a Script to Download a Bundle Adjustment in the Large (BAL) Dataset 
```shell
./sample_bal_dataset.sh
```

### Compille and Run
```shell
cd bundle_adjustment_on_manifold
mkdir build && cd build
cmake ..
make -j4

# Run the executable after successful compilation
./bundle_adjuster_on_manifold
```