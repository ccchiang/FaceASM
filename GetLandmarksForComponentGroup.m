function landmarks = GetLandmarksForComponentGroup(groupid)
global ComponentGroups;
global LandmarkGroups;
verticeGroups = ComponentGroups{groupid};
landmarks = [];
if verticeGroups==0
    landmarks = 1:64;
else
    for vg = 1:length(verticeGroups)
        landmarks = [landmarks LandmarkGroups{verticeGroups(vg)}];
    end
end
