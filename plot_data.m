f = fopen('tmp.bin', 'rb');
a = uint8(fread(f, [20, Inf], 'uint8'));
timestamp = typecast(reshape(a(1:4,:),[],1),'uint32');
iq = typecast(reshape(a(13:16,:),[],1),'single');

figure(1);
plot(diff(timestamp));
%ylim([17900,2*18100])

figure(2);
plot(iq);

fclose(f);