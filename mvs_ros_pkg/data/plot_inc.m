fid=fopen('inc_time');
s=textscan(fid,'%f','headerlines',23);
fclose(fid);
inc=s{1};
t = transpose(0:0.01:0.01*(length(s{1})-1));
plot(t,inc);
%axis([0 t(end) 98 102]);