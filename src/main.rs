use std::io;

fn main() {
    println!("Enter your name:");

    // Create a mutable string to store input
    let mut name = String::new();

    // Read a line from the standard input
    io::stdin()
        .read_line(&mut name)
        .expect("Failed to read line");

    // Trim the newline character and print
    println!("Hello, {}! Welcome to Rust.", name.trim());
}
