function M = loadcsvmatrix(filename)
    M = readmatrix(filename);
    if size(M,2) == 1 && any(isnan(M),"all")
        C = readcell(filename);
        M = str2double(split(string(C), ','));
    end
end