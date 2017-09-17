extern crate cpp_build;

fn main() {
    cpp_build::Config::new()
        .include("/usr/include/bullet")
        .build("src/lib.rs");

    println!("cargo:rustc-link-lib=BulletDynamics");
    println!("cargo:rustc-link-lib=BulletCollision");
    println!("cargo:rustc-link-lib=LinearMath");
}
