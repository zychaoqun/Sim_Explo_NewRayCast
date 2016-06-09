function[map]=map_setup(map_path)
    map=imread(map_path);
    map=rgb2gray(map);
    map(map>230)=255;
    map(map<=230)=0;
end