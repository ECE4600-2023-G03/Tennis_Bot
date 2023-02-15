
docker build -t stella_vslam . --build-arg NUM_THREADS=`expr $(nproc) - 1`
