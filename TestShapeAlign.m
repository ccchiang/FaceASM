close all;
load('All_Mesh.txt');
[NoOfPersons Dim] = size(All_Mesh);
ref = All_Mesh(1,:);
aligned = zeros(NoOfPersons, Dim);
plot(aligned(1,1:2:Dim), aligned(1,2:2:Dim), 'x');
axis equal;
axis ij;
hold on;
aligned(1,:) = ref;
for i=2:NoOfPersons
    data = All_Mesh(i,:);
    [aligned(i,:) v] = AlignShape(data, ref);
    plot(aligned(i,1:2:Dim), aligned(i,2:2:Dim), 'x');
end
save('aligned.txt', '-ascii', 'aligned');
m = mean(aligned);
[V D] = eig(cov(aligned));
D2=diag(sort(diag(D),'descend')); % make diagonal matrix out of sorted diagonal values of input D
[c, ind]=sort(diag(D),'descend'); % store the indices of which columns the sorted eigenvalues come from
V2=V(:,ind); % arrange the columns in this order

