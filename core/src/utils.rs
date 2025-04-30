pub fn unzip3<A, B, C>(v: Vec<(A, B, C)>) -> (Vec<A>, Vec<B>, Vec<C>) {
    let mut a = Vec::with_capacity(v.len());
    let mut b = Vec::with_capacity(v.len());
    let mut c = Vec::with_capacity(v.len());

    for (x, y, z) in v {
        a.push(x);
        b.push(y);
        c.push(z);
    }

    (a, b, c)
}
