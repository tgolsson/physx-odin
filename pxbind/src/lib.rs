pub mod consumer;
mod dump;
pub mod generator;
pub mod cgenerator;
pub use dump::*;

pub type Node = clang_ast::Node<consumer::Item>;
