```
docker build -t hd_maps -f Dockerfile.desktop .
PS C:\Users\User>  docker run -it --rm --mount type=bind,source="E:/code/2022-08-19T10_53_31.139760",target=/data hd_maps
```