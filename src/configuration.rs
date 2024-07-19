use std::ops::{Add, AddAssign, Sub, SubAssign};
use nalgebra::{ClosedAdd, ClosedDiv, ClosedMul, ClosedSub, Const, Matrix3, OVector, RealField, Storage, Vector, Vector3};

/// An affine transformation.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AffineTransform<T> {
    pub translation: Vector3<T>,
    pub transform: Matrix3<T>,
}

impl<RF: RealField> AffineTransform<RF> {
    pub fn scaled_by(&self, scale: RF) -> Self {
        Self {
            translation: self.translation.scale(scale.clone()),
            transform: self.transform.scale(scale),
        }
    }
}

impl<T: RealField, S: Storage<T, Const<12>>> From<Vector<T, Const<12>, S>> for AffineTransform<T> {
    fn from(value: Vector<T, Const<12>, S>) -> Self {
        AffineTransform {
            translation: Vector3::new(value[0].clone(), value[1].clone(), value[2].clone()),
            transform: Matrix3::new(
                value[3].clone(),
                value[4].clone(),
                value[5].clone(),
                value[6].clone(),
                value[7].clone(),
                value[8].clone(),
                value[9].clone(),
                value[10].clone(),
                value[11].clone(),
            ),
        }
    }
}

impl<T: RealField> From<AffineTransform<T>> for OVector<T, Const<12>> {
    fn from(value: AffineTransform<T>) -> Self {
        Self::from_iterator(
            value.translation.iter().cloned().chain(
                value
                    .transform
                    .row_iter()
                    .flat_map(|r| r.into_owned().data.0.into_iter().flatten()),
            ),
        )
    }
}

impl<RF: ClosedAdd + nalgebra::Scalar> Add for AffineTransform<RF> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            translation: &self.translation + &rhs.translation,
            transform: &self.transform + &rhs.transform,
        }
    }
}

impl<RF: ClosedAdd + nalgebra::Scalar> AddAssign for AffineTransform<RF> {

    fn add_assign(&mut self, rhs: Self) {
        self.translation += rhs.translation;
        self.transform += rhs.transform;
    }
}

impl<RF: ClosedSub + nalgebra::Scalar> Sub for AffineTransform<RF> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            translation: &self.translation - &rhs.translation,
            transform: &self.transform - &rhs.transform,
        }
    }
}

impl<RF: ClosedSub + nalgebra::Scalar> SubAssign for AffineTransform<RF> {

    fn sub_assign(&mut self, rhs: Self) {
        self.translation -= rhs.translation;
        self.transform -= rhs.transform;
    }
}

#[cfg(test)]
mod tests {
    use crate::configuration::AffineTransform;
    use nalgebra::{Const, Matrix3, OVector, Vector3};

    #[test]
    fn config_conversion() {
        let config = AffineTransform {
            translation: Vector3::new(1., 2., 3.),
            transform: Matrix3::new(4., 5., 6., 7., 8., 9., 10., 11., 12.),
        };
        let vec = OVector::<f32, Const<12>>::from_vec(vec![
            1., 2., 3., 4., 5., 6., 7., 8., 9., 10., 11., 12.,
        ]);
        assert_eq!(AffineTransform::from(vec), config);
        assert_eq!(OVector::<f32, Const<12>>::from(config), vec);
    }
}
