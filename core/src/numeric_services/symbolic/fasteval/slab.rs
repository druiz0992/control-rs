use fasteval::Slab;

pub enum ExprSlab {
    Scalar(Box<Slab>),
    Vector(Vec<Slab>),
    Matrix(Vec<Vec<Slab>>),
}
