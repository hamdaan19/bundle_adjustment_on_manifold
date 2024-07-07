# bundle_adjustment_on_manifold

### Running with Docker
```
cd bundle_adjustment_on_manifold
# Create the container
docker create --privileged -it -v ${PWD}:/home/bundle_adjustment_on_manifold -v /dev:/dev --name my_bundle_adjuster bundle_adjuster
# Start the container
docker start my_bundle_adjuster
# Enter into the container
docker exec -it my_bundle_adjuster bash
```