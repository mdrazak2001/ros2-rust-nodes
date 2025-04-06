use glob::glob;
use tower_lsp::jsonrpc::Result;
use tower_lsp::lsp_types::*;
use tower_lsp::{Client, LanguageServer, LspService, Server};

use std::collections::HashMap;
use std::env;

fn get_prefix_dirs() -> Vec<String> {
    let amnt_prefix = env::var("AMENT_PREFIX_PATH").unwrap_or_default();
    amnt_prefix.split(':').map(|s| s.to_string()).collect()
}

struct Backend {
    client: Client,
    definitions: HashMap<String, Location>,
}

#[tower_lsp::async_trait]
impl LanguageServer for Backend {
    async fn initialize(&self, _: InitializeParams) -> Result<InitializeResult> {
        // Define server capabilities, including go-to-definition and completion.
        Ok(InitializeResult {
            capabilities: ServerCapabilities {
                definition_provider: Some(OneOf::Left(true)),
                completion_provider: Some(CompletionOptions {
                    resolve_provider: Some(false),
                    trigger_characters: Some(vec!["<".to_string(), "\"".to_string()]),
                    ..Default::default()
                }),
                ..Default::default()
            },
            server_info: Some(ServerInfo {
                name: "Basic ROS2 Launch LSP".into(),
                version: Some("0.1.0".into()),
            }),
        })
    }

    async fn initialized(&self, _: InitializedParams) {
        self.client
            .log_message(MessageType::INFO, "LSP server initialized :)")
            .await;
    }

    async fn shutdown(&self) -> Result<()> {
        Ok(())
    }

    // Basic go-to-definition implementation.
    async fn goto_definition(
        &self,
        params: GotoDefinitionParams,
    ) -> Result<Option<GotoDefinitionResponse>> {
        // Log the incoming request.
        let position = params.text_document_position_params.position;
        let uri = params.text_document_position_params.text_document.uri;
        self.client
            .log_message(
                MessageType::INFO,
                format!("Goto definition called at {:?} in {}", position, uri),
            )
            .await;


        eprintln!("definitions size: {}", self.definitions.len());

        let token = "publisher".to_string();
        
        if let Some(location) = self.definitions.get(&token) {
            Ok(Some(GotoDefinitionResponse::Scalar(location.clone())))
        } else {
            self.client
                .log_message(
                    MessageType::WARNING,
                    format!("No definition found for token: {}", token),
                )
                .await;
            Ok(None)
        }
    }

    // Provide basic code completion suggestions.
    async fn completion(&self, _: CompletionParams) -> Result<Option<CompletionResponse>> {
        let items = vec![
            CompletionItem {
                label: "IncludeLaunchDescription".into(),
                kind: Some(CompletionItemKind::FUNCTION),
                detail: Some("Include another launch file".into()),
                documentation: Some(Documentation::String(
                    "This action includes a secondary launch file into the current launch description."
                        .into(),
                )),
                ..Default::default()
            },
            CompletionItem {
                label: "ExecuteProcess".into(),
                kind: Some(CompletionItemKind::FUNCTION),
                detail: Some("Execute an external process".into()),
                documentation: Some(Documentation::String(
                    "This action launches an external process with the given parameters.".into(),
                )),
                ..Default::default()
            },
        ];
        Ok(Some(CompletionResponse::Array(items)))
    }
}

fn construct_pattern(prefix: &str) -> String {
    format!("{}/share/**/rust/**/*.rs", prefix)
}

fn get_rust_nodes_path(prefixes: Vec<String>) -> Vec<String> {
    let mut rust_nodes_paths: Vec<String> = Vec::new();
    for prefix in prefixes {
        if(prefix.contains("rust_nodes")) {
            rust_nodes_paths.push(prefix.to_string());
        }
    }
    return rust_nodes_paths;
}

fn scan_workspace_for_definitions() -> HashMap<String, Location> {
    let mut definitions = HashMap::new();

    eprintln!("Scanning workspace for definitions...");
    let prefixes: Vec<String> = get_prefix_dirs();
    // for now lets look only for rust_nodes/rust_nodes_bringup
    let rust_node_definitions: Vec<String> = get_rust_nodes_path(prefixes);
    for path in rust_node_definitions {
        let pattern = construct_pattern(&path);
        eprintln!("Looking for Rust nodes in: {}", pattern);
        // glob to scan workspace 
        for entry in glob(&pattern).expect("Failed to read glob pattern") {
            match entry {
                Ok(path) => {
                    eprintln!();
                    eprintln!("Found Rust node: {:?}", path.display());
                    
                    // For demonstration, use file stem as token.
                    if let Some(file_stem) = path.file_stem().and_then(|s| s.to_str()) {
                        let location = Location {
                            uri: format!("file://{}", path.display()).parse().unwrap(),
                            range: Range {
                                start: Position { line: 0, character: 0 },
                                end: Position { line: 0, character: 0 },
                            },
                        };
                        definitions.insert(file_stem.to_string(), location);
                        eprintln!("Indexed token '{}' -> {}", file_stem, path.display());
                        eprintln!();
                    }
                }
                Err(e) => {
                    eprintln!("Error reading path: {:?}", e);
                }
            }
        }
    }
    definitions
}

#[tokio::main]
async fn main() {
    let stdin = tokio::io::stdin();
    let stdout = tokio::io::stdout();


    let prefixes: Vec<String> = get_prefix_dirs();
    for p in &prefixes {
        eprintln!("Found prefix directory: {}", p);
    }

    let definitions_map = scan_workspace_for_definitions();
    
    eprintln!("Total definitions indexed: {}", definitions_map.len());
    for (token, location) in &definitions_map {
        eprintln!("Token: {}, Location: {:?}", token, location);
    }
    
    let (service, socket) = LspService::new(|client| Backend {
        client,
        definitions: definitions_map,
    });

    Server::new(stdin, stdout, socket).serve(service).await;
}