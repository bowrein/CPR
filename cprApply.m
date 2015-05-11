function [p,pAll,Vs] = cprApply( Is, regModel, varargin )
% Apply multi stage pose regressor.
%
% USAGE
%  [p,pAll,Vs] = cprApply( Is, regModel, [varargin] )
%
% INPUTS
%  Is       - [w x h x nChn x N] input images
%  regModel - learned multi stage pose regressor
%  varargin - additional params (struct or name/value pairs)
%   .pInit    - [MxR] or [Mx2] initial pose (defaults to pStar)
%   .imgIds   - [Mx1] image id for each pose (optional if M==N)
%   .K        - [1] number of initial pose restarts
%   .rad      - [1] radius of Gassian Parzen window for finding mode
%   .chunk    - [inf] run pose clustering in chunks (to save memory)
%
% OUTPUTS
%  p        - [MxR] pose returned by multi stage regressor
%  pAll     - [MxRxT+1] pose returned at every stage of regression
%  Vs       - [wxh]x{T} visualization of ftr locations (if N==1)
%
% EXAMPLE
%
% See also cprTrain
%
% Cascaded Pose Regression Toolbox      Version 1.00
% Copyright 2012 Piotr Dollar.  [pdollar-at-caltech.edu]
% Please email me if you find bugs, or have suggestions or questions!
% Licensed under the Simplified BSD License [see bsd.txt]

dfs={'pInit',[], 'imgIds',[], 'K',1, 'rad',1, 'chunk',inf };
[p,imgIds,K,rad,chunk] = getPrmDflt(varargin,dfs,1);
if(isempty(imgIds)), imgIds=1:size(Is,4); end; M=length(imgIds);
if(isempty(p)), p=regModel.pStar(ones(M,1),1:2); end

if( K==1 )
  % Run regressor starting from a single pose.
  if(size(p,2)==2), p=[p regModel.pStar(ones(M,1),3:end)]; end
  if(nargout<3), [p,pAll]=cprApply1(Is,regModel,p,imgIds);
  else [p,pAll,Vs]=cprApply1(Is,regModel,p,imgIds); end
elseif( rad==0 )
  % Run regressor starting from mulitple poses.
  assert(nargout<=2); assert(size(p,1)==M && size(p,2)==2);
  [p,pAll] = cprApplyK(Is,regModel,p,imgIds,K);
else
  % Run regressor starting from mulitple poses and find mode.
  assert(nargout<=2); assert(size(p,1)==M && size(p,2)==2);
  T=regModel.T; R=size(regModel.pDstr,2); p0=p;
  if(M<chunk), [p,pAll]=cprApplyC(Is,regModel,p0,imgIds,K,rad); return; end
  p=zeros(M,R); pAll=zeros(M,R,T+1); s=0;
  while( s<M ), e=min(s+chunk,M);
    [p1,pAll1]=cprApplyC(Is,regModel,p0(s+1:e,:),imgIds(s+1:e),K,rad);
    p(s+1:e,:)=p1; pAll(s+1:e,:,:)=pAll1; s=e;
  end
end

end

function [p,pAll,Vs] = cprApply1( Is, regModel, p, imgIds )
% Apply each single stage regressor starting from pose p.
model=regModel.model; T=regModel.T; M=length(imgIds);
pAll=p(:,:,ones(T+1,1)); Vs=cell(1,T);
for t=1:T
  reg=regModel.regs(t); r=reg.r;
  if(nargout<3), ftrs=poseGt('ftrsComp',model,p,Is,reg.ftrData,imgIds);
  else [ftrs,Vs{t}]=poseGt('ftrsComp',model,p,Is,reg.ftrData,imgIds); end
  del = fernsRegApply(ftrs,reg.ferns);
  pDel=poseGt('identity',model,M); pDel(:,r)=del;
  p=poseGt('compose',model,p,pDel); pAll(:,:,t+1)=p;
end
end

function [p,pAll] = cprApplyK( Is, regModel, p0, imgIds, K )
% Run regressor starting from mulitple poses.
T=regModel.T; M=length(imgIds); R=size(regModel.pDstr,2);
K0=size(regModel.pDstr,1); K=min(K,K0+1);
p=repmat([regModel.pStar; regModel.pDstr(randSample(K0,K-1),:)],[M 1]);
p0=reshape(repmat(p0,1,K)',2,K*M)';
p(:,1:2)=p(:,1:2)-regModel.pStar(ones(M*K,1),1:2)+p0;
imgIds=repmat(imgIds',[1 K])'; imgIds=imgIds(:)';
[p,pAll]=cprApply1(Is,regModel,p,imgIds);
p=permute(reshape(p,K,M,R),[2 3 1]);
pAll=permute(reshape(pAll,K,M,R,T+1),[2 3 4 1]);
end

function [p,pAll] = cprApplyC( Is, regModel, p0, imgIds, K, rad )
% Run regressor starting from mulitple poses and find mode.
T=regModel.T; M=length(imgIds); R=size(regModel.pDstr,2);
[d,pAllK]=cprApplyK(Is,regModel,p0,imgIds,K); pAll=zeros(M,R,T+1);
for i=1:M, pAi=squeeze(pAllK(i,:,end,:))';
  D=poseGt('dist2',regModel.model,pAi); eD=exp(-D/(rad^2));
  [d,idx]=max(sum(eD)); pAll(i,:,:)=pAllK(i,:,:,idx);
end; p=pAll(:,:,end);
end
