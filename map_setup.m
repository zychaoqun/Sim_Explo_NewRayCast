function[map]=map_setup(map_path)
    map=imread(map_path);
    map = map.*(round(255/max(map(:))));
    if(length(size(map))>2)
        map=rgb2gray(map);
    end
    map(map>128)=255;
    map(map<128)=0;
end