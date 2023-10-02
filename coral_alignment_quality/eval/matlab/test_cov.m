clear all
close all
clc
r=2;

stddev=(r)/ sqrt(12);
x=-r:2*r/50:r;
pnts=zeros(3,size(x,2)*size(x,2)*size(x,2));
count = 0;
for i=1:size(x,2)
    for j=1:size(x,2)
        for k=1:size(x,2)
            pnt=[x(i) x(j) x(k)]';
            if(norm(pnt)<=r)
                count=count+1;
                pnts(1:3,count) = pnt;
            end
        end
    end
end
%Q=cov(pnts*pnts')/size(pnts,2);
Q=cov(pnts');%/size(pnts,2)
sq=sqrt(Q(1,1));
%((2*sq)^2)^3
d=det(Q)
%d_test=4/3*pi*(stddev^2)^3
d_test=4/3*pi*(r^2/12)^3
I=1/2*log(2*pi*exp(1)*d)-1/2*log(2*pi*exp(1)*d_test)

%plot3(pnts(1,:),pnts(2,:),pnts(3,:),'*')