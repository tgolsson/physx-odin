use crate::consumer::{
    functions::{FuncBindingExt, Param},
    AstConsumer, Block, Builtin, Comment, DefinitionData, EnumVariant, FieldBinding, FlagsBinding,
    PhysxInvoke, QualType, Tag,
};

use std::collections::BTreeMap;

#[derive(serde::Deserialize, serde::Serialize)]
pub struct BlockValue {
    b: Vec<String>,
}

impl<'ast> From<&Block<'ast>> for BlockValue {
    fn from(value: &Block) -> Self {
        Self {
            b: value.b.iter().map(|s| (*s).to_owned()).collect(),
        }
    }
}

impl BlockValue {
    #[inline]
    pub fn as_slice(&self) -> &[String] {
        self.b.as_slice()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.b.is_empty()
    }
}

#[derive(serde::Deserialize, serde::Serialize)]
pub struct CommentValue {
    /// This is the top line, we attempt to retrieve this if the comment is a
    /// doxygen `/brief` section
    pub summary: BlockValue,
    /// Additional information outside of the brief section, note that the content
    /// in this paragraph will be massaged so that doxygen style # references
    /// will be transformed into appropriate rustdoc intralinks
    pub additional: BlockValue,
}

impl<'ast> From<&Comment<'ast>> for CommentValue {
    fn from(value: &Comment) -> Self {
        Self {
            summary: (&value.summary).into(),
            additional: (&value.additional).into(),
        }
    }
}
#[derive(Debug, Clone, PartialEq, serde::Deserialize, serde::Serialize)]
pub enum QualTypeValue {
    Pointer {
        is_const: bool,
        is_pointee_const: bool,
        is_array_like: bool,
        pointee: Box<QualTypeValue>,
    },
    Reference {
        is_const: bool,
        pointee: Box<QualTypeValue>,
    },
    Builtin(Builtin),
    FunctionPointer,
    Array {
        element: Box<QualTypeValue>,
        len: u32,
    },
    Enum {
        name: String,
        cxx_qt: String,
        repr: Builtin,
    },
    Flags {
        name: String,
        repr: Builtin,
    },
    Record {
        name: String,
    },
    TemplateTypedef {
        name: String,
    },
}

impl QualTypeValue {
    // #[inline]
    // pub fn rust_type(&self) -> RustType<'_> {
    //     RustType(self)
    // }

    #[inline]
    pub fn is_pod(&self) -> bool {
        match self {
            QualTypeValue::Pointer { pointee, .. } => pointee.is_pod(),
            QualTypeValue::Builtin(bi) => bi.is_pod(),
            QualTypeValue::FunctionPointer => false,
            QualTypeValue::Array { element, .. } => element.is_pod(),
            QualTypeValue::Enum { .. }
            | QualTypeValue::Flags { .. }
            | QualTypeValue::Record { .. }
            | QualTypeValue::Reference { .. }
            | QualTypeValue::TemplateTypedef { .. } => true,
        }
    }
}

#[derive(Clone, PartialEq, serde::Deserialize, serde::Serialize)]
pub struct FieldBindingValue {
    pub name: String,
    pub kind: QualTypeValue,
    pub is_public: bool,
    pub is_reference: bool,
}

impl<'ast> From<&FieldBinding<'ast>> for FieldBindingValue {
    fn from(value: &FieldBinding<'ast>) -> Self {
        let FieldBinding {
            name,
            kind,
            is_public,
            is_reference,
        } = value;

        Self {
            name: (*name).to_owned(),
            kind: kind.into(),
            is_public: *is_public,
            is_reference: *is_reference,
        }
    }
}

#[derive(Clone, serde::Deserialize, serde::Serialize)]
pub struct RecBindingDef {
    pub name: String,
    pub has_vtable: bool,
    pub fields: Vec<FieldBindingValue>,
    pub bases: Vec<RecBindingDef>,
    pub base_fields: Vec<FieldBindingValue>,
    pub calc_layout: bool,
    pub ast_tag_used: Option<Tag>,
    pub def_data: Option<DefinitionData>,
}

impl<'ast> From<&crate::consumer::RecBindingDef<'ast>> for RecBindingDef {
    fn from(value: &crate::consumer::RecBindingDef<'ast>) -> Self {
        let crate::consumer::RecBindingDef {
            name,
            has_vtable,
            ast,
            fields,
            bases,
            base_fields,
            calc_layout,
        } = value;

        Self {
            name: (*name).to_owned(),
            has_vtable: *has_vtable,
            fields: fields.iter().map(|v| v.into()).collect(),
            bases: bases.iter().map(|v| v.into()).collect(),
            base_fields: base_fields.iter().map(|v| v.into()).collect(),
            calc_layout: *calc_layout,
            ast_tag_used: ast.tag_used,
            def_data: ast.definition_data.clone(),
        }
    }
}

#[derive(serde::Deserialize, serde::Serialize)]
pub struct RecBindingForward {
    pub name: String,
}

impl<'ast> From<&crate::consumer::RecBindingForward<'ast>> for RecBindingForward {
    fn from(value: &crate::consumer::RecBindingForward<'ast>) -> Self {
        let crate::consumer::RecBindingForward { name } = value;

        Self {
            name: (*name).to_owned(),
        }
    }
}

#[derive(serde::Deserialize, serde::Serialize)]
pub enum RecBindingValue {
    Forward(RecBindingForward),
    Def(RecBindingDef),
}

impl RecBindingValue {
    pub fn name(&self) -> &str {
        match self {
            Self::Def(def) => &def.name,
            Self::Forward(fwd) => &fwd.name,
        }
    }
}

#[derive(serde::Deserialize, serde::Serialize)]
pub struct EnumVariantValue {
    /// The name of the variant
    pub name: String,
    /// The constant value of the variant
    pub value: i64,
    /// Text comment on the enum constant
    pub comment: Option<CommentValue>,
}

impl<'ast> From<&EnumVariant<'ast>> for EnumVariantValue {
    fn from(value: &EnumVariant<'ast>) -> Self {
        let EnumVariant {
            name,
            value,
            comment,
        } = value;
        Self {
            name: (*name).to_owned(),
            value: *value,
            comment: comment.as_ref().map(|v| v.into()),
        }
    }
}

#[derive(serde::Deserialize, serde::Serialize)]
pub struct EnumBindingValue {
    /// The repr() applied to the the Rust enum to get it the correct size
    pub repr: Builtin,
    /// The "friendly" name of the enum, eg `PxErrorCode`
    pub name: String,
    /// The qualified type of the enum, minus the physx:: namespace since all
    /// the c/cpp code we compile is done within that namespace, eg. PxErrorCode::Enum
    pub cxx_qt: String,
    /// Text comment on the enum (or more usually, the wrapping struct)
    pub comment: Option<CommentValue>,
    /// The list of constants
    pub variants: Vec<EnumVariantValue>,
}

#[derive(serde::Deserialize, serde::Serialize)]
pub struct FlagsBindingValue {
    /// The name of the typedef used in the public API
    pub name: String,
    /// The index in the AstConsumer of the enum binding
    pub enums_index: usize,
    /// The storage type used by the flags
    pub storage_type: Builtin,
}

#[derive(serde::Deserialize, serde::Serialize)]
pub enum PhysxInvokeValue {
    /// Normal function call. Not many of these in PhysX, it's pretty OO
    Func { func_name: String, is_c: bool },
    /// Method call
    Method {
        func_name: String,
        class_name: String,
        is_static: bool,
    },
    /// Constructor call
    Ctor(String),
    /// New
    New(String),
}

impl<'ast> From<&PhysxInvoke<'ast>> for PhysxInvokeValue {
    fn from(value: &PhysxInvoke<'ast>) -> Self {
        match value {
            PhysxInvoke::Func { func_name, is_c } => Self::Func {
                func_name: (*func_name).to_owned(),
                is_c: *is_c,
            },
            PhysxInvoke::Method {
                func_name,
                class_name,
                is_static,
            } => Self::Method {
                func_name: (*func_name).to_owned(),
                class_name: (*class_name).to_owned(),
                is_static: *is_static,
            },
            PhysxInvoke::Ctor(s) => Self::Ctor((*s).to_owned()),
            PhysxInvoke::New(s) => Self::New((*s).to_owned()),
        }
    }
}

#[derive(serde::Deserialize, serde::Serialize)]
pub enum FuncBindingExtValue {
    IsDelete(String),
    None(PhysxInvokeValue),
    HasSelf(PhysxInvokeValue),
}

impl<'ast> From<&FuncBindingExt<'ast>> for FuncBindingExtValue {
    fn from(value: &FuncBindingExt<'ast>) -> Self {
        match value {
            FuncBindingExt::IsDelete(v) => FuncBindingExtValue::IsDelete((*v).to_owned()),
            FuncBindingExt::None(v) => FuncBindingExtValue::None(v.into()),
            FuncBindingExt::HasSelf(v) => FuncBindingExtValue::HasSelf(v.into()),
        }
    }
}

#[derive(serde::Deserialize, serde::Serialize)]
pub struct FuncBindingValue {
    pub name: String,
    pub comment: Option<CommentValue>,
    pub ext: FuncBindingExtValue,
    pub params: Vec<ParamValue>,
    pub ret: Option<QualTypeValue>,
}

#[derive(serde::Deserialize, serde::Serialize)]
pub struct ParamValue {
    pub name: String,
    pub kind: QualTypeValue,
}

impl<'ast> From<&Param<'ast>> for ParamValue {
    fn from(value: &Param<'ast>) -> Self {
        Self {
            name: (*value.name).to_owned(),
            kind: (&value.kind).into(),
        }
    }
}
#[derive(serde::Deserialize, serde::Serialize)]
pub struct TypeDB {
    pub enums: Vec<EnumBindingValue>,
    pub enum_map: BTreeMap<String, (Builtin, String)>,
    pub flags: Vec<FlagsBindingValue>,
    pub flags_map: BTreeMap<String, Builtin>,
    pub type_defs: BTreeMap<String, QualTypeValue>,
    pub recs: Vec<RecBindingValue>,
    pub funcs: Vec<FuncBindingValue>,
    pub classes: BTreeMap<String, bool>,
    pub derived: BTreeMap<String, Vec<String>>,
}

impl<'ast> From<&crate::consumer::EnumBinding<'ast>> for EnumBindingValue {
    fn from(value: &crate::consumer::EnumBinding<'ast>) -> Self {
        let crate::consumer::EnumBinding {
            repr,
            name,
            cxx_qt,
            comment,
            variants,
        } = value;

        Self {
            repr: *repr,
            name: (*name).to_owned(),
            cxx_qt: (*cxx_qt).to_owned(),
            comment: comment.as_ref().map(|v| v.into()),
            variants: variants.iter().map(|v| v.into()).collect(),
        }
    }
}

impl<'ast> From<&crate::consumer::FlagsBinding<'ast>> for FlagsBindingValue {
    fn from(value: &crate::consumer::FlagsBinding<'ast>) -> Self {
        let FlagsBinding {
            name,
            enums_index,
            storage_type,
        } = value;

        Self {
            name: (*name).to_owned(),
            enums_index: *enums_index,
            storage_type: *storage_type,
        }
    }
}

impl<'ast> From<&crate::consumer::QualType<'ast>> for QualTypeValue {
    fn from(value: &crate::consumer::QualType<'ast>) -> Self {
        match value {
            QualType::Pointer {
                is_const,
                is_pointee_const,
                is_array_like,
                pointee,
            } => QualTypeValue::Pointer {
                is_const: *is_const,
                is_pointee_const: *is_pointee_const,
                is_array_like: *is_array_like,
                pointee: Box::new(pointee.as_ref().into()),
            },
            QualType::Reference { is_const, pointee } => QualTypeValue::Reference {
                is_const: *is_const,
                pointee: Box::new(pointee.as_ref().into()),
            },
            QualType::Builtin(v) => QualTypeValue::Builtin(*v),
            QualType::FunctionPointer => QualTypeValue::FunctionPointer,
            QualType::Array { element, len } => QualTypeValue::Array {
                element: Box::new(element.as_ref().into()),
                len: *len,
            },
            QualType::Enum { name, cxx_qt, repr } => QualTypeValue::Enum {
                name: (*name).to_owned(),
                cxx_qt: (*cxx_qt).to_owned(),
                repr: *repr,
            },
            QualType::Flags { name, repr } => QualTypeValue::Flags {
                name: (*name).to_owned(),
                repr: *repr,
            },
            QualType::Record { name } => QualTypeValue::Record {
                name: (*name).to_owned(),
            },
            QualType::TemplateTypedef { name } => QualTypeValue::TemplateTypedef {
                name: (*name).to_owned(),
            },
        }
    }
}

impl<'ast> From<&crate::consumer::RecBinding<'ast>> for RecBindingValue {
    fn from(value: &crate::consumer::RecBinding<'ast>) -> Self {
        match value {
            crate::consumer::RecBinding::Forward(fwd) => Self::Forward(fwd.into()),
            crate::consumer::RecBinding::Def(def) => Self::Def(def.into()),
        }
    }
}

impl<'ast> From<&crate::consumer::FuncBinding<'ast>> for FuncBindingValue {
    fn from(value: &crate::consumer::FuncBinding<'ast>) -> Self {
        let crate::consumer::FuncBinding {
            name,
            comment,
            ext,
            params,
            ret,
        } = value;

        Self {
            name: name.into(),
            comment: comment.as_ref().map(|c| c.into()),
            ext: ext.into(),
            params: params.iter().map(|v| v.into()).collect(),
            ret: ret.as_ref().map(|v| v.into()),
        }
    }
}

impl<'ast> From<&AstConsumer<'ast>> for TypeDB {
    fn from(value: &AstConsumer<'ast>) -> Self {
        Self {
            enums: value.enums.iter().map(|v| v.into()).collect(),
            enum_map: value
                .enum_map
                .iter()
                .map(|(k, (b, s))| ((*k).to_owned(), (*b, (*s).to_owned())))
                .collect(),
            flags: value.flags.iter().map(|v| v.into()).collect(),
            flags_map: value
                .flags_map
                .iter()
                .map(|(k, v)| ((*k).to_owned(), v.to_owned()))
                .collect(),
            type_defs: value
                .type_defs
                .iter()
                .map(|(k, v)| ((*k).to_owned(), v.into()))
                .collect(),
            recs: value.recs.iter().map(|v| v.into()).collect(),
            funcs: value.funcs.iter().map(|v| v.into()).collect(),
            classes: value
                .classes
                .iter()
                .map(|(k, v)| ((*k).to_owned(), v.is_some()))
                .collect(),
            derived: value
                .derived
                .iter()
                .map(|v| ((*v.0).to_owned(), v.1.to_owned()))
                .collect(),
        }
    }
}
