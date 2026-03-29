function yq = getCurve(A,nbar,xq)
    headers = A(1,:);

    valid = ~isnan(headers);
    headerVals = headers(valid);
    headerCols = find(valid);

    if isempty(headerVals)
        error('No valid header values found in first row.');
    end

    [~,idx] = min(abs(headerVals - nbar));
    col = headerCols(idx);

    if col == size(A,2)
        error('Matched header is in last column, so there is no paired y-column.');
    end

    x = A(2:end,col);
    y = A(2:end,col+1);

    good = ~isnan(x) & ~isnan(y);
    x = x(good);
    y = y(good);

    [~,ix] = min(abs(x - xq));
    yq = y(ix);
end