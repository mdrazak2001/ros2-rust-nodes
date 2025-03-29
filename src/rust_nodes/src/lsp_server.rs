use tower_lsp::jsonrpc::Result;
use tower_lsp::lsp_types::*;
use tower_lsp::{Client, LanguageServer, LspService, Server};

struct Backend {
    client: Client,
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
            .log_message(MessageType::INFO, "LSP server initialized!")
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

        // Dummy example: Return a fixed location for demonstration purposes.
        let def_location = Location {
            uri: "file:///home/razak/ros2_ws/src/rust_nodes/launch/include_definition.py"
                .parse()
                .unwrap(),
            range: Range {
                start: Position { line: 9, character: 4 },
                end: Position { line: 9, character: 20 },
            },
        };

        Ok(Some(GotoDefinitionResponse::Scalar(def_location)))
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

#[tokio::main]
async fn main() {
    let stdin = tokio::io::stdin();
    let stdout = tokio::io::stdout();

    let (service, socket) = LspService::new(|client| Backend { client });
    Server::new(stdin, stdout, socket).serve(service).await;
}
