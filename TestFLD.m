clear all;
close all;
psample = load('testPSample.txt');
nsample = load('testNSample.txt');
nPoints = 64;
dim =20;
[r c] = size(psample);
nPSamples = r / 64;
[r c] = size(nsample);
nNSamples = r / 64;
testPSample=load('testPSample.txt');
testNSample=load('testNSample.txt');
[r c] = size(testPSample);
nTstPSamples = r / 64;
[r c] = size(testNSample);
nTstNSamples = r / 64;
P = zeros(nPoints, nPSamples+nTstPSamples, 20);
N = zeros(nPoints, nNSamples+nTstNSamples, 20);
PData = cell(nPoints,1);
NData = cell(nPoints,1);
tstPData = cell(nPoints,1);
tstNData = cell(nPoints,1);
V = zeros(dim, nPoints);
D = cell(nPoints,1);
muP = zeros(nPoints, dim);
muN = zeros(nPoints, dim);
mu = zeros(nPoints, dim);
er = zeros(nPoints, 1);
const = zeros(nPoints, 1);
newPData = zeros(nPoints, nPSamples+nTstPSamples);
newNData = zeros(nPoints, nNSamples+nTstNSamples);
VV = cell(nPoints, 1);
DD = cell(nPoints, 1);
hit = zeros(nPoints, 1);
reject = zeros(nPoints, 1);
hit2 = zeros(nPoints, 1);
reject2 = zeros(nPoints, 1);
for i=1:nPoints
    PData{i} = psample((i-1)*nPSamples+1:i*nPSamples,:);
    NData{i} = nsample((i-1)*nNSamples+1:i*nNSamples,:);
    tstPData{i} = testPSample((i-1)*nTstPSamples+1:i*nTstPSamples,:);
    tstNData{i} = testNSample((i-1)*nTstNSamples+1:i*nTstNSamples,:);
    T = [PData{i};tstPData{i};NData{i};tstNData{i}];
    G = [zeros(nPSamples, 1);zeros(nTstPSamples, 1);ones(nNSamples, 1);ones(nTstNSamples, 1)];
    [C,err,P,logp,coeff] = classify(T,T,G,'linear');
    V(:,i) = coeff(1,2).linear;
    er(i) = err;
    const(i) = coeff(1,2).const;
    newPData(i,:) = [V(:,i)'*PData{i}' V(:,i)'*tstPData{i}'];    
    newNData(i,:) = [V(:,i)'*NData{i}' V(:,i)'*tstNData{i}'];   
    mp = mean(newPData(i,:), 2);
    sp = std(newPData(i,:), 0, 2);
    mn = mean(newNData(i,:), 2);
    sn = std(newPData(i,:), 0, 2);
    prob1 = Gauss(newPData(i,:)', mp, sp);
    prob2 = Gauss(newPData(i,:)', mn, sn);
    hit(i) = sum(prob1>=prob2);
    prob3 = Gauss(newNData(i,:)', mp, sp);
    prob4 = Gauss(newNData(i,:)', mn, sn);
    reject(i) = sum(prob3<prob4);
    
    
    TP = [PData{i};tstPData{i}];
    TN = [NData{i};tstNData{i}];
    GG = cell(nPSamples+nTstPSamples+nNSamples+nTstNSamples,1);
    for kk = 1:nPSamples+nTstPSamples
        GG{kk} = '0';
    end
     for kk = nPSamples+nTstPSamples+1:nPSamples+nTstPSamples+nNSamples+nTstNSamples
        GG{kk} = '1';
     end
    mLDA = LDA([TP;TN], GG);
    mLDA.Compute();
    transformedPSamples = 100*mLDA.Transform(TP, 1);
    transformedNSamples =  100*mLDA.Transform(TN, 1);
    mp = mean(transformedPSamples);
    sp = std(transformedPSamples);
    mn = mean(transformedNSamples);
    sn = std(transformedNSamples);
    proba = Gauss(transformedPSamples, mp, sp);
    probb = Gauss(transformedPSamples, mn, sn);
    hit2(i) = sum(proba>=probb);
    probc = Gauss(transformedNSamples, mp, sp);
    probd = Gauss(transformedNSamples, mn, sn);
    reject2(i) = sum(probc<probd);
end

rate = (hit+reject)/(nPSamples+nNSamples+nTstPSamples+nTstNSamples);
rate2 = (hit2+reject2)/(nPSamples+nNSamples+nTstPSamples+nTstNSamples);
plot(1:length(rate), rate, 'b', 1:length(rate2), rate2,'g');
% 
% TstPData = cell(nPoints,1);
% TstNData = cell(nPoints,1);
% newTstPData = zeros(nPoints, nTstPSamples);
% newTstNData = zeros(nPoints, nTstNSamples);
% for i=1:nPoints
%     TstPData{i} = testPSample((i-1)*nTstPSamples+1:i*nTstPSamples,:);
%     TstNData{i} = testNSample((i-1)*nTstNSamples+1:i*nTstNSamples,:);
%     newTstPData(i,:) = (V(:,i)'*TstPData{i}');    
%     newTstNData(i,:) = (V(:,i)'*TstNData{i}');    
% end
% hitTst = zeros(nPoints, 1);
% rejectTst = zeros(nPoints, 1);
% for i=1:nPoints
%     prob1 = Gauss(newTstPData(i,:), mp(i), sp(i));
%     prob2 = Gauss(newTstPData(i,:), mn(i), sn(i));
%     hitTst(i) = sum(prob1>=prob2);
%     prob3 = Gauss(newTstNData(i,:), mp(i), sp(i));
%     prob4 = Gauss(newTstNData(i,:), mn(i), sn(i));
%     rejectTst(i) = sum(prob3<prob4);
% end
% rateTst = (hitTst+rejectTst)/(nTstPSamples+nTstNSamples);
% hold on;
% plot(1:length(rateTst), rateTst, 'r');
% hold off;