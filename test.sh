docker build -t dbg .

# WARNING: this is hyper-insecure and lazy
xhost local:root

docker run --rm \
  -v ${PWD}/bags:/bags \
  -e DISPLAY=unix$DISPLAY \
  -w /rostrace \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -it \
  dbg
