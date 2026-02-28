function integer log2(input integer M);
    integer i;
begin
    log2 = 1;
    for (i = 0; 2**i <= M; i = i + 1)
        log2 = i + 1;
end
endfunction
