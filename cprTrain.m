function [regModel,pAll] = cprTrain( Is, pGt, varargin )
% Train multistage pose regressor.
%
% USAGE
%  [regModel,pAll] = cprTrain( Is, pGt, varargin )
%
% INPUTS
%  Is       - [w x h x nChn x N] input images
%  pGt      - [NxR] ground truth pose for each image
%  varargin - additional params (struct or name/value pairs)
%   .model    - ['REQ'] pose model
%   .pStar    - [] default pose
%   .T        - ['REQ'] number of stages
%   .L        - [1] data augmentation ratio
%   .fernPrm  - [] param struct for fernsRegTrain
%   .ftrPrm   - [] param struct for ftrsGen
%   .regModel - [] if specified continue training prev model
%   .verbose  - [0] if true output info to display
%
% OUTPUTS
%  regModel - learned multi stage pose regressor:
%   .model    - pose model
%   .pStar    - default pose (start for regression)
%   .pDstr    - distribution of poses (stored training poses)
%   .T        - number of stages learned
%   .regs     - [Tx1] learned single stage pose regressors:
%     .r        - dimension regressor operates on
%     .ferns    - random fern regressor
%     .ftrData  - pose indexed features
%  pAll    - [NxRx(T+1)] pose at each phase of training
%
% EXAMPLE
%
% See also cprApply
%
% Cascaded Pose Regression Toolbox      Version 1.00
% Copyright 2012 Piotr Dollar.  [pdollar-at-caltech.edu]
% Please email me if you find bugs, or have suggestions or questions!
% Licensed under the Simplified BSD License [see bsd.txt]

% get additional parameters and check dimensions
dfs={'model','REQ','pStar',[],'T','REQ','L',1,'fernPrm',[],...
  'ftrPrm',[],'regModel',[],'verbose',0};
[model,pStar,T,L,fernPrm,ftrPrm,regModel,verbose] ...
  = getPrmDflt(varargin,dfs,1);
[N,R]=size(pGt); assert(size(Is,4)==N);
if(isempty(pStar)), pStar=poseGt('compPhiStar',model,pGt); end

% augment data amount
pCur=repmat(pStar,N*L,1); imgIds=repmat(1:N,[1 L]);
for m=2:L, is=(1:N)+(m-1)*N;
  pTmp = poseGt('inverse',model,pGt(randperm(N),:));
  pTmp = poseGt('compose',model,pTmp,pCur(is,:));
  pCur(is,:) = poseGt('compose',model,pGt,pTmp);
end; pGt=repmat(pGt,[L 1]); N1=N; N=N*L;

% remaining initialization, possibly continue training previous
pAll = zeros(N1,R,T+1);
regs = repmat(struct('r',0,'ferns',[],'ftrData',[]),T,1);
if(isempty(regModel)), t0=1; pAll(:,:,1)=pCur(1:N1,:); else
  t0=regModel.T+1; regs(1:regModel.T)=regModel.regs;
  [d,pAll1]=cprApply(Is,regModel,'imgIds',imgIds,'pInit',pCur);
  pAll(:,:,1:t0)=pAll1(1:N1,:,:); pCur=pAll1(:,:,end);
end
loss = mean(poseGt('dist',model,pCur,pGt));
if(verbose), fprintf('t=%i/%i       loss=%f\n',t0-1,T,loss); end

% loop and gradually improve pCur
for t=t0:T
  % get target value for pose
  pTar = poseGt('inverse',model,pCur);
  pTar = poseGt('compose',model,pTar,pGt);
  %figure(1); hist(pTar); drawnow;
  
  % generate and compute pose indexed features
  ftrData = poseGt('ftrsGen',model,ftrPrm);
  ftrs = poseGt('ftrsComp',model,pCur,Is,ftrData,imgIds);
  
  % train independent 1D regressors for each r, keep best
  for r=1:R, if(r==1), best={}; end
    if(max(pTar(:,r))-min(pTar(:,r))<1e-10), continue; end
    try [ferns,del] = fernsRegTrain(ftrs,pTar(:,r),fernPrm);
    catch, continue; end %#ok<CTCH>
    pDel = poseGt('identity',model,N); pDel(:,r)=del;
    pTmp = poseGt('compose',model,pCur,pDel);
    lossTmp = mean(poseGt('dist',model,pTmp,pGt));
    if(lossTmp<loss), loss=lossTmp; best={r,ferns,pTmp}; end
  end
  
  % stop if loss did not decrease
  if(isempty(best)), T=t-1; break; end
  [r,ferns,pCur]=deal(best{:}); pAll(:,:,t+1)=pCur(1:N1,:);
  
  % perform features selection
  [ftrData,ferns.fids]=poseGt('ftrsSubset',model,ftrData,ferns.fids);
  
  % store result
  regs(t) = struct('r',r,'ferns',ferns,'ftrData',ftrData);
  if(verbose), fprintf('t=%i/%i (r=%i) loss=%f\n',t,T,r,loss); end
  if(loss<1e-5), T=t; break; end
end

% create output structure
regs=regs(1:T); pAll=pAll(:,:,1:T+1);
regModel = struct('model',model,'pStar',pStar,...
  'pDstr',pGt(1:N/L,:),'T',T,'regs',regs);

end
