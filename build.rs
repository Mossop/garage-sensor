fn main() {
    println!("cargo:rustc-link-arg-bins=-Tlinkall.x");
    #[cfg(feature = "defmt")]
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
